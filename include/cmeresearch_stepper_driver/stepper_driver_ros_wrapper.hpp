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

#ifndef STEPPER_DRIVER_ROS_WRAPPER_HPP
#define STEPPER_DRIVER_ROS_WRAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <vector>
#include "cmeresearch_msgs/msg/tinker_stepper_command.hpp"
#include "cmeresearch_msgs/msg/tinker_stepper_feedback.hpp"
#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"


static inline const char * BoolToString(const bool b)
{
    return b ? "true" : "false";
}

class SilentStepperDriverWrapper
{
public:
    //! Constructor
    SilentStepperDriverWrapper() = delete;
    SilentStepperDriverWrapper(const rclcpp::Node::SharedPtr &node, bool myparam);
    virtual ~SilentStepperDriverWrapper() = default;

    //! Initialize the driver node with parameters
    bool initialize();

private:

  rclcpp::Node::SharedPtr node_;
  bool myparam_;
  //! Subscription object for the "drive_input" topic
  rclcpp::Subscription<cmeresearch_msgs::msg::TinkerStepperCommand>::SharedPtr sub_drive_cmd_;

  void drive_callback(const cmeresearch_msgs::msg::TinkerStepperCommand::SharedPtr msg);

  //! Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr timer_;

    // for publisher
    void timer_callback();

    //! Publisher object for the "drive_output" topic
    rclcpp::Publisher<cmeresearch_msgs::msg::TinkerStepperFeedback>::SharedPtr stepper_feedback_publisher_;

    cmeresearch_msgs::msg::TinkerStepperFeedback stepper_feedback_msg_;

    std::unique_ptr<StepperDriver> stepper_driver_;
    //! Parameters

    std::string bricklet_host_;
    int bricklet_port_;
	std::string brick_position_;
    int step_resolution_;
    bool interpolation_;
    int acceleration_;
    int deceleration_;
    int steps_per_revolution_;
    bool mirror_direction_;
    int gear_ratio_;
    int max_step_vel_;
    std::string wheel_name_;
    bool hw_simulation_;

    // Parameters for motor configuration
    int standstill_current_;
    int motor_run_current_;
    int standstill_delay_time_;
    int power_down_time_;
    int stealth_threshold_;
    int coolstep_threshold_;
    int classic_threshold_;
    bool high_velocity_chopper_mode_;

    int stepper_state_;
};

#endif // STEPPER_DRIVER_ROS_WRAPPER_HPP
