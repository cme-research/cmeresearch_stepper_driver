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

#include <chrono>
#include <string>
#include <functional>
//#include <rclcpp/rclcpp.hpp>
//#include "unico_msgs/msg/drive.hpp"
//#include "unico_msgs/msg/drive_feedback.hpp"
#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"
#include "cmeresearch_stepper_driver/stepper_driver_ros_wrapper.hpp"


using namespace std::chrono_literals;

SilentStepperDriverWrapper::SilentStepperDriverWrapper(const rclcpp::Node::SharedPtr &node, bool myparam)
    : node_(node), myparam_(myparam)
{

}

bool SilentStepperDriverWrapper::initialize()
{
    bricklet_host_ = node_->declare_parameter<std::string>("bricklet_host", "localhost");
    bricklet_port_ = node_->declare_parameter<int>("bricklet_port", 4223);
	brick_position_ = node_->declare_parameter<std::string>("brick_position", "A");
    step_resolution_ = node_->declare_parameter<int>("step_resolution", 8);
    interpolation_ = node_->declare_parameter<bool>("interpolation", true);
    acceleration_ = node_->declare_parameter<int>("acceleration", 1000);
    deceleration_ = node_->declare_parameter<int>("deceleration", 1000);
    steps_per_revolution_ = node_->declare_parameter<int>("steps_per_revolution", 200);
    mirror_direction_ = node_->declare_parameter<bool>("mirror_direction", false);
    gear_ratio_ = node_->declare_parameter<int>("gear_ratio", 1);
	max_step_vel_ = node_->declare_parameter<int>("max_step_vel", 1000);
    wheel_name_ = node_->declare_parameter<std::string>("wheel_name", "wheel");
    hw_simulation_ = node_->declare_parameter<bool>("hw_simulation", false);

    // Declare parameters for motor configuration
    standstill_current_ = node_->declare_parameter<int>("standstill_current", 200);
    motor_run_current_ = node_->declare_parameter<int>("motor_run_current", 800);
    standstill_delay_time_ = node_->declare_parameter<int>("standstill_delay_time", 300);
    power_down_time_ = node_->declare_parameter<int>("power_down_time", 5000);
    stealth_threshold_ = node_->declare_parameter<int>("stealth_threshold", 400);
    coolstep_threshold_ = node_->declare_parameter<int>("coolstep_threshold", 500);
    classic_threshold_ = node_->declare_parameter<int>("classic_threshold", 600);
    high_velocity_chopper_mode_ = node_->declare_parameter<bool>("high_velocity_chopper_mode", false);

    bricklet_host_ = node_->get_parameter("bricklet_host").as_string();
    bricklet_port_ = node_->get_parameter("bricklet_port").as_int();
	brick_position_ = node_->get_parameter("brick_position").as_string();
    step_resolution_ = node_->get_parameter("step_resolution").as_int();
    interpolation_ = node_->get_parameter("interpolation").as_bool();
    acceleration_ = node_->get_parameter("acceleration").as_int();
    deceleration_ = node_->get_parameter("deceleration").as_int();
    steps_per_revolution_ = node_->get_parameter("steps_per_revolution").as_int();
    mirror_direction_ = node_->get_parameter("mirror_direction").as_bool();
    gear_ratio_ = node_->get_parameter("gear_ratio").as_int();
    max_step_vel_ = node_->get_parameter("max_step_vel").as_int();
    wheel_name_ = node_->get_parameter("wheel_name").as_string();
    hw_simulation_ = node_->get_parameter("hw_simulation").as_bool();

    // Get parameters for motor configuration
    standstill_current_ = node_->get_parameter("standstill_current").as_int();
    motor_run_current_ = node_->get_parameter("motor_run_current").as_int();
    standstill_delay_time_ = node_->get_parameter("standstill_delay_time").as_int();
    power_down_time_ = node_->get_parameter("power_down_time").as_int();
    stealth_threshold_ = node_->get_parameter("stealth_threshold").as_int();
    coolstep_threshold_ = node_->get_parameter("coolstep_threshold").as_int();
    classic_threshold_ = node_->get_parameter("classic_threshold").as_int();
    high_velocity_chopper_mode_ = node_->get_parameter("high_velocity_chopper_mode").as_bool();

    RCLCPP_INFO(node_->get_logger(), "bricklet_host : %s", bricklet_host_.c_str());
    RCLCPP_INFO(node_->get_logger(), "bricklet_port : %d", bricklet_port_);
	RCLCPP_INFO(node_->get_logger(), "brick_position : %s", brick_position_.c_str());
    RCLCPP_INFO(node_->get_logger(), "step_resolution : %d", step_resolution_);
    RCLCPP_INFO(node_->get_logger(), "interpolation : %s", BoolToString(interpolation_));
    RCLCPP_INFO(node_->get_logger(), "acceleration : %d", acceleration_);
    RCLCPP_INFO(node_->get_logger(), "deceleration : %d", deceleration_);
    RCLCPP_INFO(node_->get_logger(), "steps_per_revolution : %d", steps_per_revolution_);
    RCLCPP_INFO(node_->get_logger(), "mirror_direction : %s", BoolToString(mirror_direction_));
    RCLCPP_INFO(node_->get_logger(), "gear_ratio : %d", gear_ratio_);
	RCLCPP_INFO(node_->get_logger(), "max_step_vel : %d", max_step_vel_);
    RCLCPP_INFO(node_->get_logger(), "wheel_name : %s", wheel_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "hw_simulation : %s", BoolToString(hw_simulation_));

    // Log motor configuration parameters
    RCLCPP_INFO(node_->get_logger(), "standstill_current : %d", standstill_current_);
    RCLCPP_INFO(node_->get_logger(), "motor_run_current : %d", motor_run_current_);
    RCLCPP_INFO(node_->get_logger(), "standstill_delay_time : %d", standstill_delay_time_);
    RCLCPP_INFO(node_->get_logger(), "power_down_time : %d", power_down_time_);
    RCLCPP_INFO(node_->get_logger(), "stealth_threshold : %d", stealth_threshold_);
    RCLCPP_INFO(node_->get_logger(), "coolstep_threshold : %d", coolstep_threshold_);
    RCLCPP_INFO(node_->get_logger(), "classic_threshold : %d", classic_threshold_);
    RCLCPP_INFO(node_->get_logger(), "high_velocity_chopper_mode : %s", BoolToString(high_velocity_chopper_mode_));

    std::vector<rclcpp::Parameter> new_params{
        rclcpp::Parameter("bricklet_host", bricklet_host_),
        rclcpp::Parameter("bricklet_port", bricklet_port_),
		rclcpp::Parameter("brick_position", brick_position_),
        rclcpp::Parameter("step_resolution", step_resolution_),
        rclcpp::Parameter("interpolation", interpolation_),
        rclcpp::Parameter("acceleration", acceleration_),
        rclcpp::Parameter("deceleration", deceleration_),
        rclcpp::Parameter("steps_per_revolution", steps_per_revolution_),
        rclcpp::Parameter("mirror_direction", mirror_direction_),
		rclcpp::Parameter("gear_ratio", gear_ratio_),
        rclcpp::Parameter("max_step_vel", max_step_vel_),
        rclcpp::Parameter("wheel_name", wheel_name_),
        rclcpp::Parameter("hw_simulation", hw_simulation_),
        // Motor configuration parameters
        rclcpp::Parameter("standstill_current", standstill_current_),
        rclcpp::Parameter("motor_run_current", motor_run_current_),
        rclcpp::Parameter("standstill_delay_time", standstill_delay_time_),
        rclcpp::Parameter("power_down_time", power_down_time_),
        rclcpp::Parameter("stealth_threshold", stealth_threshold_),
        rclcpp::Parameter("coolstep_threshold", coolstep_threshold_),
        rclcpp::Parameter("classic_threshold", classic_threshold_),
        rclcpp::Parameter("high_velocity_chopper_mode", high_velocity_chopper_mode_)
    };

//    auto topic_callback = [this](unico_msgs::msg::Drive::UniquePtr msg) -> void {
//      RCLCPP_INFO(node_->get_logger(), "I heard: '%f : %f '", msg->drivers[0], msg->drivers[1]);
//    };

    node_->set_parameters(new_params);

    //stepper_feedback_msg_ = std::make_shared<unico_msgs::msg::UnicoStepperFeedback()>;
//    StepperDriver stepper(bricklet_host_, bricklet_port_);
	stepper_feedback_publisher_ = node_->create_publisher<cmeresearch_msgs::msg::TinkerStepperFeedback>("drive_output", 10);

	sub_drive_cmd_ = node_->create_subscription<cmeresearch_msgs::msg::TinkerStepperCommand>("drive_input", 10, std::bind(&SilentStepperDriverWrapper::drive_callback, this, std::placeholders::_1));


	stepper_driver_ = std::make_unique<StepperDriver>(
            bricklet_host_,
            bricklet_port_,
            brick_position_,
            step_resolution_,
            interpolation_,
            acceleration_,
            deceleration_,
            steps_per_revolution_,
            gear_ratio_,
            max_step_vel_,
            standstill_current_,
            motor_run_current_,
            standstill_delay_time_,
            power_down_time_,
            stealth_threshold_,
            coolstep_threshold_,
            classic_threshold_,
            high_velocity_chopper_mode_);

    timer_ = node_->create_wall_timer(100ms, std::bind(&SilentStepperDriverWrapper::timer_callback, this));
    if (stepper_driver_->init() == 0) {
      RCLCPP_INFO(node_->get_logger(), "StepperDriver: Initialized the StepperDriverWrapper. Finished. Waiting for commands.");
      return true;
	}

    RCLCPP_INFO(node_->get_logger(), "StepperDriver: Failed to initialize the StepperDriver");
    return false;
}

void SilentStepperDriverWrapper::drive_callback(const cmeresearch_msgs::msg::TinkerStepperCommand::SharedPtr msg) {
  //RCLCPP_INFO(node_->get_logger(), "With values : %f, %f", msg->interface_values[0], msg->values[1]);
  //TODO: We need to know which drivers index to choose (left wheel or right wheel)
  //TODO: find out if wheel should go forward or backward!
  double cmd_vel = msg->velocity;

  	if (hw_simulation_) {
	    // rad/s --> revolutions/s
    	double revolutions_per_sec = (msg->velocity * gear_ratio_) / (2 * M_PI);
  		uint16_t steps_per_sec = std::round(revolutions_per_sec * steps_per_revolution_ * step_resolution_);
    	RCLCPP_INFO(node_->get_logger(), "Simulation mode: cmd_vel in steps/s= %d", steps_per_sec);
    }
	else {
    	if (mirror_direction_) {
      		cmd_vel *= -1;
    	}

		if (cmd_vel >= 0) {
  			stepper_driver_->drive_forward();
        	stepper_driver_->set_velocity(cmd_vel);
		}
		else if (cmd_vel < 0) {
  			stepper_driver_->drive_backward();
        	stepper_driver_->set_velocity(abs(cmd_vel));
		}
    }
}


void SilentStepperDriverWrapper::timer_callback() {
  	stepper_state_ = stepper_driver_->get_new_state();
    int prev_state = stepper_driver_->get_previous_state();
    // TODO: save state to recognice direction change?
	stepper_feedback_msg_.header.stamp = node_->now();
    stepper_feedback_msg_.current_velocity = stepper_driver_->get_current_velocity();
    stepper_feedback_msg_.current_position = stepper_driver_->get_current_position();

    stepper_feedback_publisher_->publish(stepper_feedback_msg_);
}
