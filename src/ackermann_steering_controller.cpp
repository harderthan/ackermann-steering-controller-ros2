/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Masaru Morita, Bence Magyar, Enrique Fernández, Manuel Meraz, Kwanghoe Kim
 */

#include "ackermann_steering_controller/ackermann_steering_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace ackermann_steering_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

AckermannSteeringController::AckermannSteeringController() : controller_interface::ControllerInterface() {}

controller_interface::return_type AckermannSteeringController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("left_wheel_name", std::string());
    auto_declare<std::string>("right_wheel_name", std::string());

    auto_declare<std::string>("rear_wheel_name", std::string());
    auto_declare<std::string>("front_steer_name", std::string());

    auto_declare<double>("wheel_separation", wheel_params_.separation);
    auto_declare<int>("wheels_per_side", wheel_params_.wheels_per_side);
    auto_declare<double>("wheel_radius", wheel_params_.radius);
    auto_declare<double>("wheel_separation_multiplier", wheel_params_.separation_multiplier);
    auto_declare<double>("left_wheel_radius_multiplier", wheel_params_.left_radius_multiplier);
    auto_declare<double>("right_wheel_radius_multiplier", wheel_params_.right_radius_multiplier);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", NAN);
    auto_declare<double>("angular.z.min_acceleration", NAN);
    auto_declare<double>("angular.z.max_jerk", NAN);
    auto_declare<double>("angular.z.min_jerk", NAN);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

InterfaceConfiguration AckermannSteeringController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.emplace_back(rear_wheel_name_ + "/" + HW_IF_VELOCITY);
  conf_names.emplace_back(front_steer_name_ + "/" + HW_IF_VELOCITY);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration AckermannSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.emplace_back(rear_wheel_name_ + "/" + HW_IF_POSITION);
  conf_names.emplace_back(front_steer_name_ + "/" + HW_IF_POSITION);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type AckermannSteeringController::update()
{
  auto logger = node_->get_logger();
  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }
  
  const auto current_time = node_->get_clock()->now();

  std::shared_ptr<Twist> last_msg;
  received_velocity_msg_ptr_.get(last_msg);

  if (last_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto dt = current_time - last_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (dt > cmd_vel_timeout_)
  {
    last_msg->twist.linear.x = 0.0;
    last_msg->twist.angular.z = 0.0;
  }

  // linear_command and angular_command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  double linear_command = last_msg->twist.linear.x;
  double angular_command = last_msg->twist.angular.z;

  // Apply (possibly new) multipliers:
  const auto wheels = wheel_params_;
  // const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  // const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  // const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  if (odom_params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command, angular_command, current_time);
  }
  else
  {
    double left_position_mean = 0.0;
    double right_position_mean = 0.0;
    // TODO: Ackermann Steering Controller haven't   wheels_per_side
    for (size_t index = 0; index < wheels.wheels_per_side; ++index)
    {
      const double left_position = registered_rear_wheel_handle_[index].position.get().get_value();
      const double right_position = registered_front_steer_handle_[index].position.get().get_value();
      if (std::isnan(left_position) || std::isnan(right_position))
      {
        RCLCPP_ERROR(
          logger, "Either the left or right wheel position is invalid for index [%zu]", index);
        return controller_interface::return_type::ERROR;
      }
      left_position_mean += left_position;
      right_position_mean += right_position;
    }
    left_position_mean /= wheels.wheels_per_side;
    right_position_mean /= wheels.wheels_per_side;    
    
    odometry_.update(left_position_mean, right_position_mean, current_time);
  }
  
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (realtime_odometry_publisher_->trylock())
  {
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = current_time;
    odometry_message.pose.pose.position.x = odometry_.getX();
    odometry_message.pose.pose.position.y = odometry_.getY();
    odometry_message.pose.pose.orientation.x = orientation.x();
    odometry_message.pose.pose.orientation.y = orientation.y();
    odometry_message.pose.pose.orientation.z = orientation.z();
    odometry_message.pose.pose.orientation.w = orientation.w();
    odometry_message.twist.twist.linear.x = odometry_.getLinear();
    odometry_message.twist.twist.angular.z = odometry_.getAngular();
    realtime_odometry_publisher_->unlockAndPublish();
  }

  if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
  {
    auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
    transform.header.stamp = current_time;
    transform.transform.translation.x = odometry_.getX();
    transform.transform.translation.y = odometry_.getY();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    realtime_odometry_transform_publisher_->unlockAndPublish();
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(
    linear_command, last_command.linear.x, second_to_last_command.linear.x, update_dt.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, update_dt.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(*last_msg);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist.linear.x = linear_command;
    limited_velocity_command.twist.angular.z = angular_command;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // TODO: Compute wheels velocities:
  //   const double velocity_rear =
  //   const double velocity_front =
  //   const double position_front = 
  const double wheel_vel = linear_command/wheel_params_.radius; // omega = linear_vel / radius

  // TODO: Set wheels velocities:
  for (size_t index = 0; index < wheels.wheels_per_side; ++index)
  {
    //   registered_rear_wheel_handle.velocity.get().set_value(velocity_rear);
    registered_rear_wheel_handle_[index].velocity.get().set_value(wheel_vel); // set on velocity ?
    registered_front_steer_handle_[index].velocity.get().set_value(angular_command);
    //   registered_front_steer_handle.position.get().set_value(position_front);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn AckermannSteeringController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  // update parameters
  rear_wheel_name_ = node_->get_parameter("rear_wheel_name").as_string();
  front_steer_name_ = node_->get_parameter("front_steer_name").as_string();

  // Check size of wheels 
    //   if (left_wheel_names_.size() != right_wheel_names_.size())
    //   {
    //     RCLCPP_ERROR(
    //       logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
    //       left_wheel_names_.size(), right_wheel_names_.size());
    //     return CallbackReturn::ERROR;
    //   }

  if (rear_wheel_name_.empty() || front_steer_name_.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return CallbackReturn::ERROR;
  }

  wheel_params_.separation = node_->get_parameter("wheel_separation").as_double();
  wheel_params_.wheels_per_side =
    static_cast<size_t>(node_->get_parameter("wheels_per_side").as_int());
  wheel_params_.radius = node_->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier =
    node_->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier =
    node_->get_parameter("left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier =
    node_->get_parameter("right_wheel_radius_multiplier").as_double();

  const auto wheels = wheel_params_;

  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  // const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  odometry_.setWheelParams(wheel_separation, left_wheel_radius); // TODO: (double wheel_separation_h, double wheel_radius)
  odometry_.setVelocityRollingWindowSize(
    node_->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = node_->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = node_->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = node_->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = node_->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = node_->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = node_->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = node_->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_ = SpeedLimiter(
      node_->get_parameter("linear.x.has_velocity_limits").as_bool(),
      node_->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      node_->get_parameter("linear.x.has_jerk_limits").as_bool(),
      node_->get_parameter("linear.x.min_velocity").as_double(),
      node_->get_parameter("linear.x.max_velocity").as_double(),
      node_->get_parameter("linear.x.min_acceleration").as_double(),
      node_->get_parameter("linear.x.max_acceleration").as_double(),
      node_->get_parameter("linear.x.min_jerk").as_double(),
      node_->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring linear speed limiter: %s", e.what());
  }

  try
  {
    limiter_angular_ = SpeedLimiter(
      node_->get_parameter("angular.z.has_velocity_limits").as_bool(),
      node_->get_parameter("angular.z.has_acceleration_limits").as_bool(),
      node_->get_parameter("angular.z.has_jerk_limits").as_bool(),
      node_->get_parameter("angular.z.min_velocity").as_double(),
      node_->get_parameter("angular.z.max_velocity").as_double(),
      node_->get_parameter("angular.z.min_acceleration").as_double(),
      node_->get_parameter("angular.z.max_acceleration").as_double(),
      node_->get_parameter("angular.z.min_jerk").as_double(),
      node_->get_parameter("angular.z.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  // TODO
  // left and right sides are both equal at this point
  wheel_params_.wheels_per_side = 1; // left_wheel_names_.size();

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      node_->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

// initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = node_->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            node_->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = node_->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Write fake header in the stored stamped command
        std::shared_ptr<Twist> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = node_->get_clock()->now();
      });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

  previous_update_timestamp_ = node_->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSteeringController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto rear_result =
    configure_side("rear", rear_wheel_name_, registered_rear_wheel_handle_);
  const auto front_result =
    configure_side("front", front_steer_name_, registered_front_steer_handle_);

  if (rear_result == CallbackReturn::ERROR || front_result == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }

  if (registered_rear_wheel_handle_.empty() || registered_front_steer_handle_.empty())
  {
    RCLCPP_ERROR(
        node_->get_logger(), "Either left wheel interfaces, right wheel interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSteeringController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSteeringController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckermannSteeringController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool AckermannSteeringController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_rear_wheel_handle_.clear();
  registered_front_steer_handle_.clear();
  
  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn AckermannSteeringController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void AckermannSteeringController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles) {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    //   wheel_handle.position.get().set_value(0.0); // No need?
    }
  };

  halt_wheels(registered_rear_wheel_handle_);
  halt_wheels(registered_front_steer_handle_);
}

// TODO: Update
CallbackReturn AckermannSteeringController::configure_side(
  const std::string & side, const std::string & wheel_name,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = node_->get_logger();

  if (wheel_name.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return CallbackReturn::ERROR;
  }

  const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_name](const auto &interface)
      { return interface.get_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_POSITION; });

  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
    return CallbackReturn::ERROR;
  }

  const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto &interface)
      {
        return interface.get_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
    return CallbackReturn::ERROR;
  }

  registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});

  return CallbackReturn::SUCCESS;
}
} // ackermann_steering_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  ackermann_steering_controller::AckermannSteeringController, controller_interface::ControllerInterface)
