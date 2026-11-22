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

#include "cmeresearch_stepper_driver/stepper_driver_ros_wrapper.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "cmeresearch_stepper_driver/stepper_math.hpp"

using namespace std::chrono_literals;

namespace cmeresearch_stepper_driver
{

SilentStepperDriverWrapper::SilentStepperDriverWrapper(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
}

bool SilentStepperDriverWrapper::initialize()
{
  auto & n = *node_;
  auto & c = config_;

  c.host = n.declare_parameter<std::string>("bricklet_host", "localhost");
  c.port = n.declare_parameter<int>("bricklet_port", 4223);
  const auto brick_position = n.declare_parameter<std::string>("brick_position", "a");
  c.microsteps = n.declare_parameter<int>("step_resolution", 8);
  c.interpolation = n.declare_parameter<bool>("interpolation", true);
  c.acceleration = n.declare_parameter<int>("acceleration", 1000);
  c.deceleration = n.declare_parameter<int>("deceleration", 1000);
  c.steps_per_revolution = n.declare_parameter<int>("steps_per_revolution", 200);
  c.max_step_vel = n.declare_parameter<int>("max_step_vel", 1000);
  mirror_direction_ = n.declare_parameter<bool>("mirror_direction", false);
  gear_ratio_ = n.declare_parameter<double>("gear_ratio", 1.0);
  wheel_name_ = n.declare_parameter<std::string>("wheel_name", "wheel");
  hw_simulation_ = n.declare_parameter<bool>("hw_simulation", false);

  c.standstill_current = n.declare_parameter<int>("standstill_current", 200);
  c.motor_run_current = n.declare_parameter<int>("motor_run_current", 800);
  c.standstill_delay_time = n.declare_parameter<int>("standstill_delay_time", 300);
  c.power_down_time = n.declare_parameter<int>("power_down_time", 5000);
  c.stealth_threshold = n.declare_parameter<int>("stealth_threshold", 400);
  c.coolstep_threshold = n.declare_parameter<int>("coolstep_threshold", 500);
  c.classic_threshold = n.declare_parameter<int>("classic_threshold", 600);
  c.high_velocity_chopper_mode = n.declare_parameter<bool>("high_velocity_chopper_mode", false);
  c.freewheel_mode = n.declare_parameter<int>("freewheel_mode", 0);
  c.under_voltage_mv = n.declare_parameter<int>("under_voltage_mv", 17000);
  c.under_voltage_hysteresis_mv = n.declare_parameter<int>("under_voltage_hysteresis_mv", 500);

  const auto logger = n.get_logger();

  if (brick_position.size() != 1) {
    RCLCPP_ERROR(logger, "brick_position must be a single letter a..h (got '%s')", brick_position.c_str());
    publish_state("error");
    return false;
  }
  c.brick_position = brick_position[0];
  if (!(gear_ratio_ > 0.0) || !std::isfinite(gear_ratio_)) {
    RCLCPP_ERROR(logger, "gear_ratio must be > 0 (got %f)", gear_ratio_);
    return false;
  }
  if (c.steps_per_revolution <= 0) {
    RCLCPP_ERROR(logger, "steps_per_revolution must be > 0 (got %d)", c.steps_per_revolution);
    return false;
  }
  if (!step_resolution_to_register(c.microsteps)) {
    RCLCPP_ERROR(
      logger, "step_resolution must be one of 1, 2, 4, ..., 256 (got %d)", c.microsteps);
    return false;
  }

  RCLCPP_INFO(
    logger,
    "[%s] brickd %s:%d port '%c' | %d steps/rev x %d microsteps (interpolation %s), "
    "gear %.3f, mirror %s, max %d steps/s, accel/decel %d/%d%s",
    wheel_name_.c_str(), c.host.c_str(), c.port, c.brick_position, c.steps_per_revolution,
    c.microsteps, c.interpolation ? "on" : "off", gear_ratio_, mirror_direction_ ? "yes" : "no",
    c.max_step_vel, c.acceleration, c.deceleration, hw_simulation_ ? " | HW SIMULATION" : "");
  RCLCPP_INFO(
    logger,
    "[%s] current run/standstill %d/%d mA, standstill delay %d, power down %d, "
    "thresholds stealth/coolstep/classic %d/%d/%d, hv chopper %s, freewheel %d, "
    "under-voltage %d mV (+%d hysteresis)",
    wheel_name_.c_str(), c.motor_run_current, c.standstill_current, c.standstill_delay_time,
    c.power_down_time, c.stealth_threshold, c.coolstep_threshold, c.classic_threshold,
    c.high_velocity_chopper_mode ? "on" : "off", c.freewheel_mode, c.under_voltage_mv,
    c.under_voltage_hysteresis_mv);

  state_pub_ = n.create_publisher<std_msgs::msg::String>("state", rclcpp::QoS(1).transient_local());
  feedback_pub_ = n.create_publisher<cmeresearch_msgs::msg::TinkerStepperFeedback>("drive_output", 10);

  if (!hw_simulation_) {
    try {
      driver_ = std::make_unique<StepperDriver>(
        config_, [logger](LogLevel level, const std::string & msg) {
          switch (level) {
            case LogLevel::Debug: RCLCPP_DEBUG(logger, "%s", msg.c_str()); break;
            case LogLevel::Info: RCLCPP_INFO(logger, "%s", msg.c_str()); break;
            case LogLevel::Warn: RCLCPP_WARN(logger, "%s", msg.c_str()); break;
            case LogLevel::Error: RCLCPP_ERROR(logger, "%s", msg.c_str()); break;
          }
        });
    } catch (const std::invalid_argument & e) {
      RCLCPP_ERROR(logger, "Invalid stepper configuration: %s", e.what());
      publish_state("error");
      return false;
    }
    if (driver_->connect() != 0) {
      publish_state("error");
      return false;
    }
  }

  sub_drive_cmd_ = n.create_subscription<cmeresearch_msgs::msg::TinkerStepperCommand>(
    "drive_input", 10,
    std::bind(&SilentStepperDriverWrapper::drive_callback, this, std::placeholders::_1));
  timer_ = n.create_wall_timer(100ms, std::bind(&SilentStepperDriverWrapper::timer_callback, this));

  publish_state("initialized");
  RCLCPP_INFO(logger, "[%s] Stepper driver ready, waiting for commands", wheel_name_.c_str());
  return true;
}

void SilentStepperDriverWrapper::publish_state(const std::string & state)
{
  if (!state_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = state;
  state_pub_->publish(msg);
}

void SilentStepperDriverWrapper::drive_callback(
  const cmeresearch_msgs::msg::TinkerStepperCommand::SharedPtr msg)
{
  if (!std::isfinite(msg->velocity)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000, "Ignoring non-finite velocity command");
    return;
  }
  // Wheel rad/s (joint frame) -> motor-shaft rad/s.
  const double joint_motor_rad_s = msg->velocity * gear_ratio_;

  if (hw_simulation_) {
    const int32_t magnitude = rad_s_to_step_velocity(
      joint_motor_rad_s, config_.steps_per_revolution, config_.microsteps, config_.max_step_vel);
    sim_velocity_ = joint_motor_rad_s < 0.0 ? -magnitude : magnitude;
    return;
  }

  // mirror_direction corrects for motors mounted / wired the other way round.
  driver_->command_velocity(mirror_direction_ ? -joint_motor_rad_s : joint_motor_rad_s);
}

void SilentStepperDriverWrapper::timer_callback()
{
  cmeresearch_msgs::msg::TinkerStepperFeedback msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = wheel_name_;

  bool low_voltage = false;
  bool recovered = false;
  if (hw_simulation_) {
    msg.current_velocity = static_cast<float>(sim_velocity_);
  } else {
    const StepperFeedback fb = driver_->feedback();
    // Undo mirror_direction so the sign matches the command convention.
    msg.current_velocity = static_cast<float>(mirror_direction_ ? -fb.velocity : fb.velocity);
    msg.current_position = fb.position;
    msg.remaining_steps = fb.remaining_steps;
    msg.input_voltage = fb.input_voltage_mv;
    msg.current_consumption = fb.current_consumption_ma;
    low_voltage = fb.under_voltage;
    recovered = driver_->consume_voltage_recovered();
  }
  feedback_pub_->publish(msg);

  if (low_voltage && !prev_low_voltage_) {
    publish_state("low_voltage");
    RCLCPP_WARN(node_->get_logger(), "[%s] Low supply voltage", wheel_name_.c_str());
  }
  if (recovered) {
    publish_state("idle");
    RCLCPP_INFO(node_->get_logger(), "[%s] Supply voltage recovered, state -> idle", wheel_name_.c_str());
  }
  prev_low_voltage_ = low_voltage;
}

}  // namespace cmeresearch_stepper_driver
