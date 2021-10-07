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
 * Author: Masaru Morita, Bence Magyar, Enrique Fernández, Kwanghoe Kim
 */

#ifndef ACKERMANN_STEERING_CONTROLLER__ACKERMANN_STEERING_CONTROLLER_HPP_
#define ACKERMANN_STEERING_CONTROLLER__ACKERMANN_STEERING_CONTROLLER_HPP_

#include "ackermann_steering_controller/odometry.hpp"
#include "ackermann_steering_controller/speed_limiter.hpp"
#include "ackermann_steering_controller/visibility_control.h"

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace ackermann_steering_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class AckermannSteeringController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;
public:
  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  AckermannSteeringController();

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_STEERING_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  CallbackReturn configure_side(
    const std::string & side, const std::string & wheel_name,
    std::vector<WheelHandle> & registered_handles);
    
  std::string rear_wheel_name_;
  std::string front_steer_name_;

  std::vector<WheelHandle> registered_rear_wheel_handle_;
  std::vector<WheelHandle> registered_front_steer_handle_;

  struct WheelParams
  {
    size_t wheels_per_side = 0;
    double separation = 0.0;  // w.r.t. the midpoint of the wheel width
    double radius = 0.0;      // Assumed to be the same for both wheels
    double separation_multiplier = 1.0;
    double left_radius_multiplier = 1.0;
    double right_radius_multiplier = 1.0;
  } wheel_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace ackermann_steering_controller
#endif // ACKERMANN_STEERING_CONTROLLER__ACKERMANN_STEERING_CONTROLLER_HPP_