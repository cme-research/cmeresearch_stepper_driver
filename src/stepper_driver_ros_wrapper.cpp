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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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

SilentStepperDriverWrapper::~SilentStepperDriverWrapper()
{
  // execute() leaves its loop once rclcpp is shut down.
  if (execute_thread_.joinable()) {
    execute_thread_.join();
  }
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
  const auto control_mode = declare<std::string>(
    n, "control_mode", "velocity",
    "velocity: drive_input topic (wheels); position: move_steps action (actuators)");
  const double command_timeout = declare<double>(
    n, "command_timeout", 0.5,
    "velocity mode: stop if no command arrives for this long [s]; 0 disables");
  const auto robot_state_topic = declare<std::string>(
    n, "robot_state_topic", "/robot_state",
    "position mode: abort moves on emergency_stop from this RobotState topic; empty disables");

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
  if (control_mode != "velocity" && control_mode != "position") {
    return fail("control_mode must be 'velocity' or 'position' (got '" + control_mode + "')");
  }
  position_mode_ = control_mode == "position";
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
  // Position moves run without a command stream; the watchdog only guards
  // the continuous-drive (velocity) mode.
  watchdog_ = CommandWatchdog(position_mode_ ? 0.0 : command_timeout);

  RCLCPP_INFO(
    logger,
    "[%s] %s mode | brickd %s:%d port '%c' | %d steps/rev x %d microsteps (interpolation %s), "
    "gear %.3f, mirror %s, max %d steps/s, accel/decel %d/%d%s",
    wheel_name_.c_str(), control_mode.c_str(), c.host.c_str(), c.port, c.brick_position,
    c.steps_per_revolution, c.microsteps, c.interpolation ? "on" : "off", gear_ratio_,
    mirror_direction_ ? "yes" : "no", c.max_step_vel, c.acceleration, c.deceleration,
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

  if (position_mode_) {
    using std::placeholders::_1;
    using std::placeholders::_2;
    move_server_ = rclcpp_action::create_server<MoveSteps>(
      node_, "move_steps",
      std::bind(&SilentStepperDriverWrapper::handle_goal, this, _1, _2),
      std::bind(&SilentStepperDriverWrapper::handle_cancel, this, _1),
      std::bind(&SilentStepperDriverWrapper::handle_accepted, this, _1));
    zero_srv_ = n.create_service<std_srvs::srv::Trigger>(
      "zero_position",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::string err;
        res->success = zero_position(&err);
        res->message = res->success ? "current position is now 0" : err;
      });
    if (!robot_state_topic.empty()) {
      robot_state_sub_ = n.create_subscription<cmeresearch_msgs::msg::RobotState>(
        robot_state_topic, rclcpp::QoS(1).transient_local().reliable(),
        [this](const cmeresearch_msgs::msg::RobotState::SharedPtr msg) {
          const bool stop = msg->state == "emergency_stop";
          if (stop && !emergency_stop_.exchange(true)) {
            RCLCPP_WARN(node_->get_logger(), "[%s] Emergency stop: aborting / refusing moves",
              wheel_name_.c_str());
          } else if (!stop) {
            emergency_stop_.store(false);
          }
        });
    }
  } else {
    sub_drive_cmd_ = n.create_subscription<cmeresearch_msgs::msg::TinkerStepperCommand>(
      "drive_input", 10,
      std::bind(&SilentStepperDriverWrapper::drive_callback, this, std::placeholders::_1));
  }

  last_tick_s_ = now_s();
  timer_ = n.create_wall_timer(100ms, std::bind(&SilentStepperDriverWrapper::timer_callback, this));

  RCLCPP_INFO(
    logger, "[%s] Stepper driver ready, waiting for %s", wheel_name_.c_str(),
    position_mode_ ? "move_steps goals" : "commands");
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

// --- velocity mode -----------------------------------------------------------------

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
    std::lock_guard<std::mutex> lock(sim_mutex_);
    sim_velocity_ = joint_motor_rad_s < 0.0 ? -magnitude : magnitude;
    return;
  }

  // mirror_direction corrects for motors mounted / wired the other way round.
  driver_->command_velocity(mirror_direction_ ? -joint_motor_rad_s : joint_motor_rad_s);
}

// --- position mode -------------------------------------------------------------------

rclcpp_action::GoalResponse SilentStepperDriverWrapper::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveSteps::Goal> goal)
{
  if (goal->mode != MoveSteps::Goal::RELATIVE && goal->mode != MoveSteps::Goal::ABSOLUTE) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Rejecting move: unknown mode %u",
      wheel_name_.c_str(), goal->mode);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal_active_.load()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Rejecting move: another move is running",
      wheel_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Everything else (not ready, not referenced, emergency stop) is accepted and
  // aborted in execute() so the client gets the reason in the result message.
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SilentStepperDriverWrapper::handle_cancel(
  std::shared_ptr<GoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SilentStepperDriverWrapper::handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
{
  goal_active_.store(true);
  if (execute_thread_.joinable()) {
    execute_thread_.join();   // the previous goal already finished
  }
  execute_thread_ = std::thread([this, goal_handle]() {
      execute(goal_handle);
      goal_active_.store(false);
    });
}

void SilentStepperDriverWrapper::execute(std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveSteps::Result>();
  auto finish = [&](bool success, const std::string & message) {
      const auto st = move_status();
      result->success = success;
      result->final_position = st.final_position;
      result->message = message;
      return result;
    };

  if (emergency_stop_.load()) {
    goal_handle->abort(finish(false, "emergency stop active"));
    return;
  }
  std::string err;
  if (!start_move(goal->mode == MoveSteps::Goal::ABSOLUTE, goal->steps, goal->max_velocity, &err)) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Move refused: %s", wheel_name_.c_str(), err.c_str());
    goal_handle->abort(finish(false, err));
    return;
  }

  auto feedback = std::make_shared<MoveSteps::Feedback>();
  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      cancel_move("canceled");
      goal_handle->canceled(finish(false, "canceled"));
      return;
    }
    if (emergency_stop_.load()) {
      cancel_move("emergency stop");
      goal_handle->abort(finish(false, "emergency stop"));
      return;
    }
    if (!hw_simulation_) {
      const auto status = driver_->status();
      if (status != DriverStatus::Ready) {
        const std::string why = std::string("driver ") + to_string(status);
        cancel_move(why);
        goal_handle->abort(finish(false, why));
        return;
      }
      if (driver_->feedback().under_voltage) {
        cancel_move("low supply voltage");
        goal_handle->abort(finish(false, "low supply voltage"));
        return;
      }
    }

    const auto st = move_status();
    if (st.state == MoveStatus::State::Reached) {
      goal_handle->succeed(finish(true, "reached"));
      return;
    }
    if (st.state == MoveStatus::State::Aborted) {
      goal_handle->abort(finish(false, st.reason));
      return;
    }

    if (hw_simulation_) {
      std::lock_guard<std::mutex> lock(sim_mutex_);
      feedback->current_position = static_cast<int32_t>(std::lround(sim_position_));
      feedback->current_velocity = static_cast<float>(sim_velocity_);
    } else {
      const auto fb = driver_->feedback();
      feedback->current_position = mirror() * fb.position;
      feedback->current_velocity = static_cast<float>(mirror() * fb.velocity);
    }
    feedback->remaining_steps = st.target - feedback->current_position;
    goal_handle->publish_feedback(feedback);
    rate.sleep();
  }
}

bool SilentStepperDriverWrapper::start_move(
  bool absolute, int32_t steps, uint16_t max_velocity, std::string * error)
{
  if (!hw_simulation_) {
    // The driver works in the motor frame.
    return driver_->start_move(absolute, mirror() * steps, max_velocity, error);
  }
  std::lock_guard<std::mutex> lock(sim_mutex_);
  if (sim_move_.state == MoveStatus::State::Moving) {
    *error = "another move is still running";
    return false;
  }
  if (absolute && !sim_referenced_) {
    *error = "absolute move needs a reference: call zero_position first";
    return false;
  }
  const auto start = static_cast<int32_t>(std::lround(sim_position_));
  sim_move_ = MoveStatus{};
  sim_move_.target = absolute ? steps : start + steps;
  sim_move_velocity_ = max_velocity == 0 ? config_.max_step_vel :
    std::min<int>(max_velocity, config_.max_step_vel);
  if (sim_move_.target == start) {
    sim_move_.state = MoveStatus::State::Reached;
    sim_move_.final_position = start;
  } else {
    sim_move_.state = MoveStatus::State::Moving;
  }
  return true;
}

void SilentStepperDriverWrapper::cancel_move(const std::string & reason)
{
  if (!hw_simulation_) {
    driver_->cancel_move(reason);
    return;
  }
  std::lock_guard<std::mutex> lock(sim_mutex_);
  if (sim_move_.state == MoveStatus::State::Moving) {
    sim_move_.state = MoveStatus::State::Aborted;
    sim_move_.reason = reason;
    sim_move_.final_position = static_cast<int32_t>(std::lround(sim_position_));
    sim_velocity_ = 0;
  }
}

MoveStatus SilentStepperDriverWrapper::move_status()
{
  if (!hw_simulation_) {
    // Back to the actuator frame.
    MoveStatus st = driver_->move_status();
    st.target *= mirror();
    st.final_position *= mirror();
    return st;
  }
  std::lock_guard<std::mutex> lock(sim_mutex_);
  return sim_move_;
}

bool SilentStepperDriverWrapper::zero_position(std::string * error)
{
  if (!hw_simulation_) {
    return driver_->set_reference(0, error);
  }
  std::lock_guard<std::mutex> lock(sim_mutex_);
  if (sim_move_.state == MoveStatus::State::Moving) {
    *error = "cannot set the reference while moving";
    return false;
  }
  sim_position_ = 0.0;
  sim_referenced_ = true;
  return true;
}

void SilentStepperDriverWrapper::sim_tick(double dt)
{
  std::lock_guard<std::mutex> lock(sim_mutex_);
  if (!position_mode_) {
    sim_position_ += sim_velocity_ * dt;
    return;
  }
  if (sim_move_.state != MoveStatus::State::Moving) {
    sim_velocity_ = 0;
    return;
  }
  const double remaining = sim_move_.target - sim_position_;
  const double step = sim_move_velocity_ * dt;
  if (std::abs(remaining) <= step) {
    sim_position_ = sim_move_.target;
    sim_velocity_ = 0;
    sim_move_.state = MoveStatus::State::Reached;
    sim_move_.final_position = sim_move_.target;
  } else {
    const int dir = remaining > 0 ? 1 : -1;
    sim_position_ += dir * step;
    sim_velocity_ = static_cast<int32_t>(dir * sim_move_velocity_);
  }
}

// --- both modes ------------------------------------------------------------------------

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
      std::lock_guard<std::mutex> lock(sim_mutex_);
      sim_velocity_ = 0;
    } else {
      driver_->command_velocity(0.0);
    }
  }

  cmeresearch_msgs::msg::TinkerStepperFeedback msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = wheel_name_;

  if (hw_simulation_) {
    sim_tick(dt);
    std::lock_guard<std::mutex> lock(sim_mutex_);
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
