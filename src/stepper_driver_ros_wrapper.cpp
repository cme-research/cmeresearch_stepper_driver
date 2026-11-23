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

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "cmeresearch_stepper_driver/bricklet_io.hpp"
#include "cmeresearch_stepper_driver/stepper_config.hpp"

using namespace std::chrono_literals;

namespace cmeresearch_stepper_driver
{

namespace
{

/// All parameters are read once at start-up; declaring them read-only makes
/// `ros2 param set` fail visibly instead of silently having no effect.
template<typename T>
T declare(rclcpp::Node & node, const std::string & name, const T & default_value,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  d.read_only = true;
  return node.declare_parameter<T>(name, default_value, d);
}

}  // namespace

SilentStepperDriverWrapper::SilentStepperDriverWrapper(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
}

double SilentStepperDriverWrapper::now_s() const
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool SilentStepperDriverWrapper::initialize()
{
  auto & n = *node_;
  auto & c = config_;
  const auto logger = n.get_logger();

  // Created first so even a configuration error is reported on `state`.
  state_pub_ = n.create_publisher<std_msgs::msg::String>("state", rclcpp::QoS(1).transient_local());

  c.host = declare<std::string>(n, "bricklet_host", "localhost", "brickd host");
  c.port = declare<int>(n, "bricklet_port", 4223, "brickd TCP port");
  const auto brick_position = declare<std::string>(
    n, "brick_position", "a", "Bricklet port letter on the brick (a..h)");
  c.microsteps = declare<int>(
    n, "step_resolution", 8, "Microsteps per full step: 1, 2, 4, ..., 256");
  c.interpolation = declare<bool>(n, "interpolation", true, "256-microstep interpolation");
  c.acceleration = declare<int>(n, "acceleration", 1000, "Acceleration ramp [steps/s^2], 0..65535");
  c.deceleration = declare<int>(n, "deceleration", 1000, "Deceleration ramp [steps/s^2], 0..65535");
  c.steps_per_revolution = declare<int>(
    n, "steps_per_revolution", 200, "Full steps per motor revolution");
  c.max_step_vel = declare<int>(n, "max_step_vel", 1000, "Velocity cap [microsteps/s], 1..65535");
  mirror_direction_ = declare<bool>(
    n, "mirror_direction", false, "Invert for motors mounted the other way round");
  gear_ratio_ = declare<double>(n, "gear_ratio", 1.0, "Motor revolutions per wheel revolution");
  wheel_name_ = declare<std::string>(n, "wheel_name", "wheel", "Log prefix and feedback frame_id");
  hw_simulation_ = declare<bool>(
    n, "hw_simulation", false, "No hardware: feedback echoes the commanded velocity");
  const double command_timeout = declare<double>(
    n, "command_timeout", 0.5,
    "Stop the wheel if no velocity command arrives for this long [s]; 0 disables");

  c.standstill_current = declare<int>(
    n, "standstill_current", 200, "Standstill current [mA], <= motor_run_current");
  c.motor_run_current = declare<int>(n, "motor_run_current", 800, "Motor run current [mA]");
  c.standstill_delay_time = declare<int>(n, "standstill_delay_time", 300, "[ms]");
  c.power_down_time = declare<int>(n, "power_down_time", 5000, "[ms]");
  c.stealth_threshold = declare<int>(n, "stealth_threshold", 400, "Stealth mode upper threshold");
  c.coolstep_threshold = declare<int>(n, "coolstep_threshold", 500, "Coolstep threshold");
  c.classic_threshold = declare<int>(n, "classic_threshold", 600, "Classic mode threshold");
  c.high_velocity_chopper_mode = declare<bool>(n, "high_velocity_chopper_mode", false, "");
  c.freewheel_mode = declare<int>(
    n, "freewheel_mode", 0,
    "Standstill behaviour when standstill_current is 0: 0 normal, 1 freewheel, 2/3 coil short");
  c.under_voltage_mv = declare<int>(
    n, "under_voltage_mv", 17000, "Supply level that triggers low_voltage [mV]");
  c.under_voltage_hysteresis_mv = declare<int>(
    n, "under_voltage_hysteresis_mv", 500, "low_voltage clears at threshold + hysteresis [mV]");
  c.feedback_timeout_s = declare<double>(
    n, "feedback_timeout", 0.5, "Feedback older than this counts as lost [s]");
  c.connect_retry_ms = declare<int>(
    n, "connect_retry_ms", 1000, "Wait between attempts to reach brickd [ms]");

  // --- validation -------------------------------------------------------------
  auto fail = [&](const std::string & msg) {
      RCLCPP_ERROR(logger, "[%s] %s", wheel_name_.c_str(), msg.c_str());
      publish_state("error");
      return false;
    };
  if (brick_position.size() != 1) {
    return fail("brick_position must be a single letter a..h (got '" + brick_position + "')");
  }
  c.brick_position = brick_position[0];
  if (!(gear_ratio_ > 0.0) || !std::isfinite(gear_ratio_)) {
    return fail("gear_ratio must be > 0 (got " + std::to_string(gear_ratio_) + ")");
  }
  if (!(command_timeout >= 0.0) || !std::isfinite(command_timeout)) {
    return fail("command_timeout must be >= 0 (got " + std::to_string(command_timeout) + ")");
  }
  const auto check = validate(c);
  for (const auto & w : check.warnings) {
    RCLCPP_WARN(logger, "[%s] %s", wheel_name_.c_str(), w.c_str());
  }
  if (!check.ok()) {
    for (const auto & e : check.errors) {
      RCLCPP_ERROR(logger, "[%s] invalid parameter: %s", wheel_name_.c_str(), e.c_str());
    }
    publish_state("error");
    return false;
  }
  watchdog_ = CommandWatchdog(command_timeout);

  RCLCPP_INFO(
    logger,
    "[%s] brickd %s:%d port '%c' | %d steps/rev x %d microsteps (interpolation %s), "
    "gear %.3f, mirror %s, max %d steps/s, accel/decel %d/%d, command timeout %.2f s%s",
    wheel_name_.c_str(), c.host.c_str(), c.port, c.brick_position, c.steps_per_revolution,
    c.microsteps, c.interpolation ? "on" : "off", gear_ratio_, mirror_direction_ ? "yes" : "no",
    c.max_step_vel, c.acceleration, c.deceleration, command_timeout,
    hw_simulation_ ? " | HW SIMULATION" : "");
  RCLCPP_INFO(
    logger,
    "[%s] current run/standstill %d/%d mA, standstill delay %d, power down %d, "
    "thresholds stealth/coolstep/classic %d/%d/%d, hv chopper %s, freewheel %d, "
    "under-voltage %d mV (+%d hysteresis)",
    wheel_name_.c_str(), c.motor_run_current, c.standstill_current, c.standstill_delay_time,
    c.power_down_time, c.stealth_threshold, c.coolstep_threshold, c.classic_threshold,
    c.high_velocity_chopper_mode ? "on" : "off", c.freewheel_mode, c.under_voltage_mv,
    c.under_voltage_hysteresis_mv);

  feedback_pub_ = n.create_publisher<cmeresearch_msgs::msg::TinkerStepperFeedback>("drive_output", 10);

  if (!hw_simulation_) {
    driver_ = std::make_unique<StepperDriver>(
      config_, make_tinkerforge_io(), [logger](LogLevel level, const std::string & msg) {
        switch (level) {
          case LogLevel::Debug: RCLCPP_DEBUG(logger, "%s", msg.c_str()); break;
          case LogLevel::Info: RCLCPP_INFO(logger, "%s", msg.c_str()); break;
          case LogLevel::Warn: RCLCPP_WARN(logger, "%s", msg.c_str()); break;
          case LogLevel::Error: RCLCPP_ERROR(logger, "%s", msg.c_str()); break;
        }
      });
    publish_state("connecting");
    // Connects in the background and keeps retrying while brickd is not up yet.
    driver_->start();
  } else {
    publish_state("initialized");
  }

  sub_drive_cmd_ = n.create_subscription<cmeresearch_msgs::msg::TinkerStepperCommand>(
    "drive_input", 10,
    std::bind(&SilentStepperDriverWrapper::drive_callback, this, std::placeholders::_1));
  last_tick_s_ = now_s();
  timer_ = n.create_wall_timer(100ms, std::bind(&SilentStepperDriverWrapper::timer_callback, this));

  RCLCPP_INFO(logger, "[%s] Stepper driver ready, waiting for commands", wheel_name_.c_str());
  return true;
}

void SilentStepperDriverWrapper::publish_state(const std::string & state)
{
  if (!state_pub_ || state == state_) {
    return;
  }
  state_ = state;
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
  watchdog_.feed(now_s(), joint_motor_rad_s != 0.0);

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
  const double now = now_s();
  const double dt = now - last_tick_s_;
  last_tick_s_ = now;

  if (watchdog_.expired(now)) {
    RCLCPP_WARN(
      node_->get_logger(), "[%s] No velocity command for %.2f s, stopping the wheel",
      wheel_name_.c_str(), watchdog_.timeout());
    if (hw_simulation_) {
      sim_velocity_ = 0;
    } else {
      driver_->command_velocity(0.0);
    }
  }

  cmeresearch_msgs::msg::TinkerStepperFeedback msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = wheel_name_;

  if (hw_simulation_) {
    sim_position_ += sim_velocity_ * dt;
    msg.current_velocity = static_cast<float>(sim_velocity_);
    msg.current_position = static_cast<int32_t>(std::lround(sim_position_));
    feedback_pub_->publish(msg);
    return;
  }

  const StepperFeedback fb = driver_->feedback();
  // Undo mirror_direction so the sign matches the command convention.
  // (feedback() reports 0 velocity whenever the data is not live.)
  msg.current_velocity = static_cast<float>(mirror_direction_ ? -fb.velocity : fb.velocity);
  msg.current_position = fb.position;
  msg.remaining_steps = fb.remaining_steps;
  msg.input_voltage = fb.input_voltage_mv;
  msg.current_consumption = fb.current_consumption_ma;
  feedback_pub_->publish(msg);

  update_state(fb.under_voltage);
}

void SilentStepperDriverWrapper::update_state(bool under_voltage)
{
  const auto status = driver_->status();
  std::string next;
  switch (status) {
    case DriverStatus::Error:
      next = "error";
      break;
    case DriverStatus::Connecting:
    case DriverStatus::WaitingForBricklet:
    case DriverStatus::Stale:
      next = was_ready_ ? "disconnected" : "connecting";
      break;
    case DriverStatus::Ready:
      if (under_voltage) {
        next = "low_voltage";
      } else if (state_ == "low_voltage" || state_ == "idle") {
        next = "idle";              // recovered from low voltage
      } else {
        next = "initialized";
      }
      was_ready_ = true;
      break;
  }
  if (next != state_) {
    const auto logger = node_->get_logger();
    if (next == "disconnected" || next == "error" || next == "low_voltage") {
      RCLCPP_WARN(logger, "[%s] state -> %s (driver %s)", wheel_name_.c_str(), next.c_str(),
        to_string(status));
    } else {
      RCLCPP_INFO(logger, "[%s] state -> %s", wheel_name_.c_str(), next.c_str());
    }
    publish_state(next);
  }
}

}  // namespace cmeresearch_stepper_driver
