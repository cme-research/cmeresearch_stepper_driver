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
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "cmeresearch_stepper_driver/stepper_math.hpp"

namespace cmeresearch_stepper_driver
{

namespace
{

const StepperConfig & require_valid(const StepperConfig & config)
{
  const auto check = validate(config);
  if (!check.ok()) {
    std::string msg = "invalid stepper configuration:";
    for (const auto & e : check.errors) {
      msg += "\n  - " + e;
    }
    throw std::invalid_argument(msg);
  }
  return config;
}

char lower(char c) {return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));}

}  // namespace

const char * to_string(DriverStatus status)
{
  switch (status) {
    case DriverStatus::Connecting: return "connecting";
    case DriverStatus::WaitingForBricklet: return "waiting_for_bricklet";
    case DriverStatus::Ready: return "ready";
    case DriverStatus::Stale: return "stale";
    case DriverStatus::Error: return "error";
  }
  return "unknown";
}

StepperDriver::StepperDriver(
  StepperConfig config, std::unique_ptr<BrickletIo> io, LogFn log, NowFn now)
: config_(require_valid(config)),
  step_resolution_register_(*step_resolution_to_register(config_.microsteps)),
  io_(std::move(io)),
  log_(std::move(log)),
  now_(std::move(now))
{
  BrickletCallbacks cb;
  cb.on_connected = [this](bool auto_reconnect) {handle_connected(auto_reconnect);};
  cb.on_disconnected = [this]() {handle_disconnected();};
  cb.on_enumerate = [this](const EnumerateInfo & info) {handle_enumerate(info);};
  cb.on_all_data = [this](uint16_t v, int32_t p, int32_t r, uint16_t u, uint16_t i) {
      handle_all_data(v, p, r, u, i);
    };
  cb.on_new_state = [this](uint8_t s, uint8_t prev) {handle_new_state(s, prev);};
  // Registered before connecting so the initial CONNECTED event (which
  // triggers enumeration) cannot be missed.
  io_->set_callbacks(std::move(cb));
}

StepperDriver::~StepperDriver()
{
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stopping_ = true;
  }
  stop_cv_.notify_all();
  if (connect_thread_.joinable()) {
    connect_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (configured_.load() && device_created_) {
      log(LogLevel::Info, "Shutting down stepper: stop and disable driver stage");
      io_->stop();
      // Give the ramp a moment before cutting the driver stage.
      std::this_thread::sleep_for(std::chrono::milliseconds(config_.shutdown_ramp_ms));
      io_->set_enabled(false);
    }
    configured_.store(false);
  }

  // Stops the bindings' threads, so no handler runs below. Must not hold
  // device_mutex_ here: a callback may be waiting on it.
  io_->disconnect();
  if (device_created_) {
    io_->destroy_device();
    device_created_ = false;
  }
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

void StepperDriver::start()
{
  if (!connect_thread_.joinable()) {
    connect_thread_ = std::thread([this]() {connect_loop();});
  }
}

void StepperDriver::connect_loop()
{
  const std::string where = config_.host + ":" + std::to_string(config_.port);
  for (int attempt = 1;; ++attempt) {
    {
      std::lock_guard<std::mutex> lock(stop_mutex_);
      if (stopping_) {
        return;
      }
    }
    const int e_code = io_->connect(config_.host, static_cast<uint16_t>(config_.port));
    if (e_code == 0) {
      // From now on the bindings reconnect on their own after a connection loss.
      io_->set_auto_reconnect(true);
      connected_.store(true);
      log(LogLevel::Info, "Connected to brickd at " + where);
      return;
    }
    // Log the first failure and then every 10th, so a missing brickd does not
    // flood the log but stays visible.
    if (attempt == 1 || attempt % 10 == 0) {
      log(
        LogLevel::Warn, "brickd at " + where + " not reachable (error " +
        std::to_string(e_code) + "), retrying every " +
        std::to_string(config_.connect_retry_ms) + " ms (attempt " + std::to_string(attempt) + ")");
    }
    std::unique_lock<std::mutex> lock(stop_mutex_);
    stop_cv_.wait_for(
      lock, std::chrono::milliseconds(config_.connect_retry_ms), [this]() {return stopping_;});
  }
}

DriverStatus StepperDriver::status() const
{
  if (config_error_.load()) {
    return DriverStatus::Error;
  }
  if (!connected_.load()) {
    return DriverStatus::Connecting;
  }
  if (!configured_.load()) {
    return DriverStatus::WaitingForBricklet;
  }
  return is_stale() ? DriverStatus::Stale : DriverStatus::Ready;
}

bool StepperDriver::is_stale() const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  const auto age = std::chrono::duration<double>(now_() - last_data_).count();
  return age > config_.feedback_timeout_s;
}

void StepperDriver::handle_connected(bool auto_reconnect)
{
  connected_.store(true);
  log(LogLevel::Info, auto_reconnect ? "Reconnected to brickd, enumerating" :
    "Connected, enumerating");
  io_->enumerate();
}

void StepperDriver::handle_disconnected()
{
  connected_.store(false);
  configured_.store(false);
  log(LogLevel::Warn, "Connection to brickd lost, waiting for reconnect");
}

void StepperDriver::handle_enumerate(const EnumerateInfo & info)
{
  if (!info.is_silent_stepper_v2 || lower(info.position) != lower(config_.brick_position)) {
    return;
  }

  std::lock_guard<std::mutex> lock(device_mutex_);

  if (info.type == EnumerateInfo::Type::Disconnected) {
    if (device_uid_ == info.uid) {
      configured_.store(false);
      log(LogLevel::Warn, "Bricklet " + info.uid + " disconnected");
    }
    return;
  }

  log(
    LogLevel::Info, "Found SilentStepper V2 " + info.uid + " at port " + info.position +
    " (firmware " + std::to_string(info.firmware[0]) + "." + std::to_string(info.firmware[1]) +
    "." + std::to_string(info.firmware[2]) + ")");
  // AVAILABLE (answer to our enumerate) and CONNECTED (bricklet was reset or
  // re-plugged, which also wipes its configuration) both need a full configure.
  configure_device_locked(info.uid);
}

void StepperDriver::configure_device_locked(const std::string & uid)
{
  if (device_created_ && device_uid_ != uid) {
    io_->destroy_device();
    device_created_ = false;
  }
  if (!device_created_) {
    io_->create_device(uid);
    device_created_ = true;
    device_uid_ = uid;
  }

  configured_.store(false);
  const int e_code = io_->configure(config_, step_resolution_register_);
  if (e_code != 0) {
    config_error_.store(true);
    log(
      LogLevel::Error, "Bricklet " + uid + " rejected the configuration (error " +
      std::to_string(e_code) + "); it will be retried on the next reconnect");
    return;
  }
  config_error_.store(false);

  // Force a disable/enable cycle on the next drive command.
  enabled_ = false;
  {
    std::lock_guard<std::mutex> fb_lock(feedback_mutex_);
    have_position_ = false;
    // Give the data stream a full timeout to start.
    last_data_ = now_();
  }
  configured_.store(true);
  log(LogLevel::Info, "Bricklet configured (" + std::to_string(config_.microsteps) + " microsteps)");
}

void StepperDriver::ensure_enabled_locked()
{
  const bool rearm = rearm_pending_.exchange(false);
  if (!enabled_ || rearm) {
    if (rearm) {
      log(LogLevel::Info, "Re-arming driver stage after under-voltage recovery");
    }
    io_->set_enabled(false);
    io_->set_enabled(true);
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
    io_->set_max_velocity(0);
    return;
  }

  ensure_enabled_locked();
  const int direction = motor_rad_s > 0.0 ? 1 : -1;
  io_->drive(direction);
  commanded_direction_.store(direction);
  io_->set_max_velocity(steps_per_s);
}

void StepperDriver::stop()
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (configured_.load()) {
    io_->stop();
  }
}

StepperFeedback StepperDriver::feedback() const
{
  const bool live = status() == DriverStatus::Ready;
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  StepperFeedback fb = feedback_;
  fb.valid = live;
  if (!live) {
    // Never hand out a frozen velocity: odometry would keep integrating it.
    fb.velocity = 0;
  }
  return fb;
}

void StepperDriver::handle_all_data(
  uint16_t velocity, int32_t position, int32_t remaining_steps,
  uint16_t input_voltage_mv, uint16_t current_consumption_ma)
{
  bool recovered = false;
  bool tripped = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    auto & fb = feedback_;
    last_data_ = now_();

    const int fallback = commanded_direction_.load();
    const int direction = have_position_ ?
      direction_from_position_delta(fb.position, position, fallback) : fallback;
    have_position_ = true;

    fb.velocity = direction * static_cast<int32_t>(velocity);
    fb.position = position;
    fb.remaining_steps = remaining_steps;
    fb.input_voltage_mv = input_voltage_mv;
    fb.current_consumption_ma = current_consumption_ma;

    const bool was_low = fb.under_voltage;
    fb.under_voltage = update_under_voltage(
      was_low, input_voltage_mv, config_.under_voltage_mv, config_.under_voltage_hysteresis_mv);
    tripped = !was_low && fb.under_voltage;
    recovered = was_low && !fb.under_voltage;
  }

  if (tripped) {
    log(LogLevel::Warn, "Input voltage low: " + std::to_string(input_voltage_mv) + " mV");
  }
  if (recovered) {
    log(LogLevel::Info, "Input voltage recovered: " + std::to_string(input_voltage_mv) + " mV");
    // The command path re-arms the driver stage on the next drive command.
    rearm_pending_.store(true);
  }
}

void StepperDriver::handle_new_state(uint8_t state_new, uint8_t state_previous)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  feedback_.state = state_new;
  feedback_.previous_state = state_previous;
}

}  // namespace cmeresearch_stepper_driver
