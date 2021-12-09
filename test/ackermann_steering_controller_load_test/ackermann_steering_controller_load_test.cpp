// Copyright 2020 PAL Robotics SL.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "common_load_test.hpp" // TODO: update common urdf for ackermann 

TEST(TestLoadAckermannSteeringController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      diff_drive_controller_testing::diffbot_urdf), // TODO: update common urdf for ackermann
    executor, "test_controller_manager");

  ASSERT_NO_THROW(
    cm.load_controller("test_ackermann_steering_controller", "ackermann_steering_controller/AckermannSteeringController"));

  rclcpp::shutdown();
}
