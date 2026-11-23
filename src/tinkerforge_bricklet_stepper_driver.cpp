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

#include "cmeresearch_stepper_driver/tinkerforge_bricklet_stepper_driver.hpp"

#include <cctype>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <utility>

#include "cmeresearch_stepper_driver/stepper_math.hpp"

namespace cmeresearch_stepper_driver
{

namespace
{

uint8_t require_step_resolution(int microsteps)
{
  const auto reg = step_resolution_to_register(microsteps);
  if (!reg) {
    throw std::invalid_argument(
            "step_resolution must be one of 1, 2, 4, 8, 16, 32, 64, 128, 256 (got " +
            std::to_string(microsteps) + ")");
  }
  return *reg;
}

// Callback signatures differ per callback id; the binding takes a generic
// function pointer and casts it back internally.
template<typename Fn>
void (* as_callback(Fn * fn))(void)
{
  return reinterpret_cast<void (*)(void)>(fn);
}

}  // namespace

StepperDriver::StepperDriver(StepperConfig config, LogFn log)
: config_(std::move(config)),
  step_resolution_register_(require_step_resolution(config_.microsteps)),
  log_(std::move(log))
{
  const char pos = static_cast<char>(std::tolower(static_cast<unsigned char>(config_.brick_position)));
  if (pos < 'a' || pos > 'h') {
    throw std::invalid_argument(
            std::string("brick_position must be a bricklet port letter a..h (got '") +
            config_.brick_position + "')");
  }
  if (config_.freewheel_mode < 0 || config_.freewheel_mode > 3) {
    throw std::invalid_argument("freewheel_mode must be in [0, 3]");
  }

  ipcon_create(&ipcon_);
  // Register before connecting so the initial CONNECTED event (which triggers
  // enumeration) cannot be missed.
  ipcon_register_callback(&ipcon_, IPCON_CALLBACK_CONNECTED, as_callback(&cb_connected), this);
  ipcon_register_callback(&ipcon_, IPCON_CALLBACK_ENUMERATE, as_callback(&cb_enumerate), this);
}

StepperDriver::~StepperDriver()
{
  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (configured_.load() && device_created_) {
      log(LogLevel::Info, "Shutting down stepper: stop and disable driver stage");
      silent_stepper_v2_stop(&device_);
      // Give the ramp a moment before cutting the driver stage.
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      silent_stepper_v2_set_enabled(&device_, false);
    }
    configured_.store(false);
  }

  // Disconnect stops the binding's threads, so no callback can run below.
  // Must not hold device_mutex_ here: a callback may be waiting on it.
  ipcon_disconnect(&ipcon_);

  if (device_created_) {
    silent_stepper_v2_destroy(&device_);
    device_created_ = false;
  }
  ipcon_destroy(&ipcon_);
}

void StepperDriver::log(LogLevel level, const std::string & msg) const
{
  if (log_) {
    log_(level, msg);
    return;
  }
  static const char * const names[] = {"DEBUG", "INFO", "WARN", "ERROR"};
  std::fprintf(stderr, "[%s] StepperDriver: %s\n", names[static_cast<int>(level)], msg.c_str());
}

int StepperDriver::connect()
{
  const int e_code = ipcon_connect(&ipcon_, config_.host.c_str(), static_cast<uint16_t>(config_.port));
  if (e_code != E_OK) {
    log(
      LogLevel::Error, "Connecting to brickd at " + config_.host + ":" +
      std::to_string(config_.port) + " failed (error " + std::to_string(e_code) + ")");
    return e_code;
  }
  // Keep reconnecting if brickd restarts; cb_connected re-enumerates and the
  // bricklet is re-configured from cb_enumerate.
  ipcon_set_auto_reconnect(&ipcon_, true);
  log(LogLevel::Info, "Connected to brickd at " + config_.host + ":" + std::to_string(config_.port));
  return E_OK;
}

void StepperDriver::cb_connected(uint8_t reason, void * user_data)
{
  auto * self = static_cast<StepperDriver *>(user_data);
  self->log(
    LogLevel::Info, reason == IPCON_CONNECT_REASON_AUTO_RECONNECT ?
    "Reconnected to brickd, enumerating" : "Connected, enumerating");
  ipcon_enumerate(&self->ipcon_);
}

void StepperDriver::cb_enumerate(
  const char * uid, const char * /*connected_uid*/, char position,
  uint8_t /*hardware_version*/[3], uint8_t firmware_version[3],
  uint16_t device_identifier, uint8_t enumeration_type, void * user_data)
{
  auto * self = static_cast<StepperDriver *>(user_data);
  if (device_identifier != SILENT_STEPPER_V2_DEVICE_IDENTIFIER) {
    return;
  }
  const auto lower = [](char c) {return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));};
  if (lower(position) != lower(self->config_.brick_position)) {
    return;
  }

  std::lock_guard<std::mutex> lock(self->device_mutex_);

  if (enumeration_type == IPCON_ENUMERATION_TYPE_DISCONNECTED) {
    if (self->device_uid_ == uid) {
      self->configured_.store(false);
      self->log(LogLevel::Warn, std::string("Bricklet ") + uid + " disconnected");
    }
    return;
  }

  self->log(
    LogLevel::Info, std::string("Found SilentStepper V2 ") + uid + " at port " + position +
    " (firmware " + std::to_string(firmware_version[0]) + "." +
    std::to_string(firmware_version[1]) + "." + std::to_string(firmware_version[2]) + ")");
  // AVAILABLE (answer to our enumerate) and CONNECTED (bricklet was reset or
  // re-plugged, which also wipes its configuration) both need a full configure.
  self->configure_device_locked(uid);
}

void StepperDriver::configure_device_locked(const char * uid)
{
  if (device_created_ && device_uid_ != uid) {
    silent_stepper_v2_destroy(&device_);
    device_created_ = false;
  }
  if (!device_created_) {
    silent_stepper_v2_create(&device_, uid, &ipcon_);
    device_created_ = true;
    device_uid_ = uid;
    silent_stepper_v2_register_callback(
      &device_, SILENT_STEPPER_V2_CALLBACK_ALL_DATA, as_callback(&cb_all_data), this);
    silent_stepper_v2_register_callback(
      &device_, SILENT_STEPPER_V2_CALLBACK_NEW_STATE, as_callback(&cb_new_state), this);
  }

  const auto & c = config_;
  silent_stepper_v2_set_step_configuration(&device_, step_resolution_register_, c.interpolation);
  silent_stepper_v2_set_motor_current(&device_, c.motor_run_current);
  silent_stepper_v2_set_max_velocity(&device_, 0);
  silent_stepper_v2_set_speed_ramping(&device_, c.acceleration, c.deceleration);
  silent_stepper_v2_set_basic_configuration(
    &device_, c.standstill_current, c.motor_run_current, c.standstill_delay_time,
    c.power_down_time, c.stealth_threshold, c.coolstep_threshold, c.classic_threshold,
    c.high_velocity_chopper_mode);
  // Only freewheel_mode deviates from the firmware defaults (stealth on,
  // amplitude 128, gradient 4, autoscale on, force_symmetric off). It governs
  // the standstill behaviour when standstill_current == 0.
  silent_stepper_v2_set_stealth_configuration(
    &device_, true, 128, 4, true, false, static_cast<uint8_t>(c.freewheel_mode));
  silent_stepper_v2_set_all_callback_configuration(&device_, 100);
  silent_stepper_v2_set_enabled(&device_, true);

  // Force a disable/enable cycle on the next drive command, as before.
  enabled_ = false;
  {
    std::lock_guard<std::mutex> fb_lock(feedback_mutex_);
    have_position_ = false;
  }
  configured_.store(true);
  log(LogLevel::Info, "Bricklet configured (" + std::to_string(c.microsteps) + " microsteps)");
}

void StepperDriver::ensure_enabled_locked()
{
  const bool rearm = rearm_pending_.exchange(false);
  if (!enabled_ || rearm) {
    if (rearm) {
      log(LogLevel::Info, "Re-arming driver stage after under-voltage recovery");
    }
    silent_stepper_v2_set_enabled(&device_, false);
    silent_stepper_v2_set_enabled(&device_, true);
    enabled_ = true;
  }
}

void StepperDriver::command_velocity(double motor_rad_s)
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (!configured_.load()) {
    return;
  }
  const uint16_t steps_per_s = rad_s_to_step_velocity(
    motor_rad_s, config_.steps_per_revolution, config_.microsteps, config_.max_step_vel);

  if (steps_per_s == 0) {
    // Ramp down in the current direction; issuing a direction call here would
    // make the bricklet run a pointless direction-change sequence.
    silent_stepper_v2_set_max_velocity(&device_, 0);
    return;
  }

  ensure_enabled_locked();
  const int direction = motor_rad_s > 0.0 ? 1 : -1;
  if (direction > 0) {
    silent_stepper_v2_drive_forward(&device_);
  } else {
    silent_stepper_v2_drive_backward(&device_);
  }
  commanded_direction_.store(direction);
  silent_stepper_v2_set_max_velocity(&device_, steps_per_s);
}

void StepperDriver::stop()
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (configured_.load()) {
    silent_stepper_v2_stop(&device_);
  }
}

StepperFeedback StepperDriver::feedback() const
{
  if (!configured_.load()) {
    return StepperFeedback{};
  }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  return feedback_;
}

void StepperDriver::cb_all_data(
  uint16_t current_velocity, int32_t current_position, int32_t remaining_steps,
  uint16_t input_voltage, uint16_t current_consumption, void * user_data)
{
  auto * self = static_cast<StepperDriver *>(user_data);
  bool recovered = false;
  bool tripped = false;
  {
    std::lock_guard<std::mutex> lock(self->feedback_mutex_);
    auto & fb = self->feedback_;

    const int fallback = self->commanded_direction_.load();
    const int direction = self->have_position_ ?
      direction_from_position_delta(fb.position, current_position, fallback) : fallback;
    self->have_position_ = true;

    fb.velocity = direction * static_cast<int32_t>(current_velocity);
    fb.position = current_position;
    fb.remaining_steps = remaining_steps;
    fb.input_voltage_mv = input_voltage;
    fb.current_consumption_ma = current_consumption;

    const bool was_low = fb.under_voltage;
    fb.under_voltage = update_under_voltage(
      was_low, input_voltage, self->config_.under_voltage_mv,
      self->config_.under_voltage_hysteresis_mv);
    tripped = !was_low && fb.under_voltage;
    recovered = was_low && !fb.under_voltage;
  }

  if (tripped) {
    self->log(LogLevel::Warn, "Input voltage low: " + std::to_string(input_voltage) + " mV");
  }
  if (recovered) {
    self->log(LogLevel::Info, "Input voltage recovered: " + std::to_string(input_voltage) + " mV");
    // Two independent consumers: the command path re-arms the driver stage,
    // the ROS wrapper publishes the state change.
    self->rearm_pending_.store(true);
    self->voltage_recovered_event_.store(true);
  }
}

void StepperDriver::cb_new_state(uint8_t state_new, uint8_t state_previous, void * user_data)
{
  auto * self = static_cast<StepperDriver *>(user_data);
  std::lock_guard<std::mutex> lock(self->feedback_mutex_);
  self->feedback_.state = state_new;
  self->feedback_.previous_state = state_previous;
}

}  // namespace cmeresearch_stepper_driver
