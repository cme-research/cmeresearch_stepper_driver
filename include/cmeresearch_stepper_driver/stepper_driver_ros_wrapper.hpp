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

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cmeresearch_msgs/action/move_steps.hpp"
#include "cmeresearch_msgs/msg/robot_state.hpp"
#include "cmeresearch_msgs/msg/tinker_stepper_command.hpp"
#include "cmeresearch_msgs/msg/tinker_stepper_feedback.hpp"
#include "cmeresearch_stepper_driver/stepper_math.hpp"
#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"

namespace cmeresearch_stepper_driver
{

/// ROS interface for one stepper.
///
/// control_mode "velocity" (default, wheels under cmexa_base):
///  - subscribes  drive_input  (TinkerStepperCommand, wheel rad/s)
///  - command watchdog stops the wheel when commands stop arriving
///
/// control_mode "position" (stand-alone actuator):
///  - action      move_steps    (cmeresearch_msgs/action/MoveSteps)
///  - service     zero_position (std_srvs/Trigger): current position := 0
///  - aborts moves on emergency_stop from robot_state_topic
///
/// Both modes:
///  - publishes   drive_output (TinkerStepperFeedback, every 100 ms)
///  - publishes   state        (std_msgs/String, latched, on change):
///      connecting | initialized | low_voltage | idle | disconnected | error
class SilentStepperDriverWrapper
{
public:
  using MoveSteps = cmeresearch_msgs::action::MoveSteps;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveSteps>;

  explicit SilentStepperDriverWrapper(rclcpp::Node::SharedPtr node);
  ~SilentStepperDriverWrapper();

  /// Read and validate parameters, set up ROS interfaces and start connecting
  /// to the hardware in the background. Returns false only for an invalid
  /// configuration; a missing brickd is retried, not fatal.
  bool initialize();

private:
  // velocity mode
  void drive_callback(const cmeresearch_msgs::msg::TinkerStepperCommand::SharedPtr msg);

  // position mode
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveSteps::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void execute(std::shared_ptr<GoalHandle> goal_handle);
  // Move backend: real driver or simulation; actuator frame (mirror applied).
  bool start_move(bool absolute, int32_t steps, uint16_t max_velocity, std::string * error);
  void cancel_move(const std::string & reason);
  MoveStatus move_status();
  bool zero_position(std::string * error);
  void sim_tick(double dt);

  void timer_callback();
  void update_state(bool under_voltage);
  void publish_state(const std::string & state);
  double now_s() const;
  int mirror() const {return mirror_direction_ ? -1 : 1;}

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr sub_drive_cmd_;
  rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<MoveSteps>::SharedPtr move_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
  rclcpp::Subscription<cmeresearch_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

  StepperConfig config_;
  std::unique_ptr<StepperDriver> driver_;
  CommandWatchdog watchdog_{0.5};

  std::string wheel_name_;
  double gear_ratio_ = 1.0;
  bool mirror_direction_ = false;
  bool hw_simulation_ = false;
  bool position_mode_ = false;

  std::string state_;          // last published state
  bool was_ready_ = false;     // reached "initialized" at least once

  // position mode
  std::atomic<bool> goal_active_{false};
  std::atomic<bool> emergency_stop_{false};
  std::thread execute_thread_;

  // hw_simulation (guarded by sim_mutex_): commanded/simulated velocity
  // [signed microsteps/s, joint/actuator frame], integrated position and, in
  // position mode, the simulated move.
  std::mutex sim_mutex_;
  int32_t sim_velocity_ = 0;
  double sim_position_ = 0.0;
  MoveStatus sim_move_;
  double sim_move_velocity_ = 0.0;
  bool sim_referenced_ = false;
  double last_tick_s_ = 0.0;
};

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__STEPPER_DRIVER_ROS_WRAPPER_HPP_
