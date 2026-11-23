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

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmeresearch_msgs/msg/tinker_stepper_command.hpp"
#include "cmeresearch_msgs/msg/tinker_stepper_feedback.hpp"
#include "cmeresearch_stepper_driver/stepper_math.hpp"
#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"

namespace cmeresearch_stepper_driver
{

/// ROS interface for one wheel:
///  - subscribes  drive_input  (TinkerStepperCommand, wheel rad/s)
///  - publishes   drive_output (TinkerStepperFeedback, every 100 ms)
///  - publishes   state        (std_msgs/String, latched, on change):
///      connecting | initialized | low_voltage | idle | disconnected | error
///
/// Feedback velocity is signed microsteps/s on the motor shaft, expressed in
/// the joint (command) direction convention, i.e. mirror_direction undone.
class SilentStepperDriverWrapper
{
public:
  explicit SilentStepperDriverWrapper(rclcpp::Node::SharedPtr node);

  /// Read and validate parameters, set up ROS interfaces and start connecting
  /// to the hardware in the background. Returns false only for an invalid
  /// configuration; a missing brickd is retried, not fatal.
  bool initialize();

private:
  void drive_callback(const cmeresearch_msgs::msg::TinkerStepperCommand::SharedPtr msg);
  void timer_callback();
  void update_state(bool under_voltage);
  void publish_state(const std::string & state);
  double now_s() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr sub_drive_cmd_;
  rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  StepperConfig config_;
  std::unique_ptr<StepperDriver> driver_;
  CommandWatchdog watchdog_{0.5};

  std::string wheel_name_;
  double gear_ratio_ = 1.0;
  bool mirror_direction_ = false;
  bool hw_simulation_ = false;

  std::string state_;          // last published state
  bool was_ready_ = false;     // reached "initialized" at least once

  /// hw_simulation only: commanded velocity [signed microsteps/s, joint frame]
  /// and the position integrated from it [microsteps].
  int32_t sim_velocity_ = 0;
  double sim_position_ = 0.0;
  double last_tick_s_ = 0.0;
};

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__STEPPER_DRIVER_ROS_WRAPPER_HPP_
