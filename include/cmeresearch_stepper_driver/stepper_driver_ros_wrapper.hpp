/*
Copyright (C) 2025 Christian Ehrmann

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef CMERESEARCH_STEPPER_DRIVER__STEPPER_DRIVER_ROS_WRAPPER_HPP_
#define CMERESEARCH_STEPPER_DRIVER__STEPPER_DRIVER_ROS_WRAPPER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmeresearch_msgs/msg/tinker_stepper_command.hpp"
#include "cmeresearch_msgs/msg/tinker_stepper_feedback.hpp"
#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"

namespace cmeresearch_stepper_driver
{

/// ROS interface for one wheel:
///  - subscribes  drive_input  (TinkerStepperCommand, wheel rad/s)
///  - publishes   drive_output (TinkerStepperFeedback, every 100 ms)
///  - publishes   state        (std_msgs/String, latched): initialized | error |
///                             low_voltage | idle
///
/// Feedback velocity is signed microsteps/s on the motor shaft, expressed in
/// the joint (command) direction convention, i.e. mirror_direction undone.
class SilentStepperDriverWrapper
{
public:
  explicit SilentStepperDriverWrapper(rclcpp::Node::SharedPtr node);

  /// Read parameters, set up ROS interfaces and connect to the hardware.
  bool initialize();

private:
  void drive_callback(const cmeresearch_msgs::msg::TinkerStepperCommand::SharedPtr msg);
  void timer_callback();
  void publish_state(const std::string & state);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr sub_drive_cmd_;
  rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  StepperConfig config_;
  std::unique_ptr<StepperDriver> driver_;

  std::string wheel_name_;
  double gear_ratio_ = 1.0;
  bool mirror_direction_ = false;
  bool hw_simulation_ = false;

  /// hw_simulation only: last commanded velocity [signed microsteps/s, joint frame].
  int32_t sim_velocity_ = 0;
  bool prev_low_voltage_ = false;
};

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__STEPPER_DRIVER_ROS_WRAPPER_HPP_
