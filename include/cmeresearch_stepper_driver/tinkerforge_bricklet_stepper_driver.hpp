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

#ifndef CMERESEARCH_STEPPER_DRIVER__TINKERFORGE_BRICKLET_STEPPER_DRIVER_HPP_
#define CMERESEARCH_STEPPER_DRIVER__TINKERFORGE_BRICKLET_STEPPER_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "cmeresearch_stepper_driver/bricklet_io.hpp"
#include "cmeresearch_stepper_driver/stepper_config.hpp"

namespace cmeresearch_stepper_driver
{

enum class LogLevel { Debug, Info, Warn, Error };
using LogFn = std::function<void(LogLevel, const std::string &)>;

/// Coarse driver status, derived on demand.
enum class DriverStatus
{
  Connecting,          ///< brickd not reached yet / connection lost
  WaitingForBricklet,  ///< connected, matching bricklet not (yet) configured
  Ready,               ///< configured and receiving live data
  Stale,               ///< configured, but no data for feedback_timeout_s
  Error,               ///< the bricklet rejected the configuration
};

const char * to_string(DriverStatus status);

/// State of a position move (control_mode "position").
struct MoveStatus
{
  enum class State { Idle, Moving, Reached, Aborted };
  State state = State::Idle;
  int32_t target = 0;          ///< expected end position (motor frame, microsteps)
  int32_t final_position = 0;  ///< position reported when the move ended
  std::string reason;          ///< why it was aborted
};

/// Drives one TinkerForge SilentStepper Bricklet 2.0 in continuous-drive mode.
///
/// Threading: BrickletIo callbacks (handle_*) run on the bindings' callback
/// thread, start() runs connection attempts on its own thread, and the public
/// command/feedback methods are called from the ROS executor. Device
/// configuration and the command path are guarded by device_mutex_, the
/// feedback snapshot by feedback_mutex_, single-word flags are atomics.
class StepperDriver
{
public:
  using Clock = std::chrono::steady_clock;
  using NowFn = std::function<Clock::time_point()>;

  /// Throws std::invalid_argument if validate(config) reports errors.
  StepperDriver(
    StepperConfig config, std::unique_ptr<BrickletIo> io, LogFn log = nullptr,
    NowFn now = &Clock::now);
  ~StepperDriver();

  StepperDriver(const StepperDriver &) = delete;
  StepperDriver & operator=(const StepperDriver &) = delete;

  /// Start connecting to brickd in the background. Attempts repeat every
  /// connect_retry_ms until brickd answers; afterwards the bindings reconnect
  /// automatically. The bricklet is configured whenever it is enumerated.
  void start();

  DriverStatus status() const;

  /// Command a signed motor-shaft velocity [rad/s] (motor frame, i.e. after
  /// gear ratio and direction mirroring). 0 ramps down to standstill.
  /// Ignored unless the bricklet is configured.
  void command_velocity(double motor_rad_s);

  /// Ramp the motor down to standstill.
  void stop();

  /// Latest data. While status() is not Ready, `valid` is false and the
  /// velocity is 0, so consumers never integrate frozen values.
  StepperFeedback feedback() const;

  // --- position control (motor frame, microsteps) -------------------------------
  /// Start a move: by `steps` (relative) or to counter position `steps`
  /// (absolute; requires is_referenced()). `max_velocity` 0 uses max_step_vel.
  /// Returns false and sets `error` if the move cannot start (not ready,
  /// another move running, not referenced, bricklet rejected it).
  bool start_move(bool absolute, int32_t steps, uint16_t max_velocity, std::string * error);
  /// Ramp down and mark the running move as aborted with `reason`.
  void cancel_move(const std::string & reason);
  MoveStatus move_status() const;
  /// Define the current position as `position` (e.g. 0 at the home position).
  bool set_reference(int32_t position, std::string * error);
  /// True once set_reference() succeeded since the bricklet was (re)configured
  /// — a bricklet reset restarts its step counter at 0.
  bool is_referenced() const {return referenced_.load();}

  // --- BrickletIo event handlers (public for tests) ---------------------------
  void handle_connected(bool auto_reconnect);
  void handle_disconnected();
  void handle_enumerate(const EnumerateInfo & info);
  void handle_all_data(
    uint16_t velocity, int32_t position, int32_t remaining_steps,
    uint16_t input_voltage_mv, uint16_t current_consumption_ma);
  void handle_new_state(uint8_t state_new, uint8_t state_previous);
  void handle_position_reached(int32_t position);

private:
  void log(LogLevel level, const std::string & msg) const;
  void abort_move(const std::string & reason);
  void connect_loop();
  void configure_device_locked(const std::string & uid);
  void ensure_enabled_locked();
  bool is_stale() const;

  const StepperConfig config_;
  const uint8_t step_resolution_register_;
  std::unique_ptr<BrickletIo> io_;
  LogFn log_;
  NowFn now_;

  // Connection thread
  std::thread connect_thread_;
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  bool stopping_ = false;

  // Guards device creation/configuration and the command path.
  mutable std::mutex device_mutex_;
  bool device_created_ = false;
  std::string device_uid_;
  bool enabled_ = false;            // driver stage enabled by us since (re)configure

  std::atomic<bool> connected_{false};
  std::atomic<bool> configured_{false};
  std::atomic<bool> config_error_{false};
  // Last direction sent to the bricklet (+1/-1); sign fallback while the
  // position is not changing.
  std::atomic<int> commanded_direction_{1};
  std::atomic<bool> rearm_pending_{false};

  mutable std::mutex feedback_mutex_;
  StepperFeedback feedback_;
  bool have_position_ = false;
  Clock::time_point last_data_{};

  // Position move. Lock order: device_mutex_ before move_mutex_.
  mutable std::mutex move_mutex_;
  MoveStatus move_;
  std::atomic<bool> referenced_{false};
};

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__TINKERFORGE_BRICKLET_STEPPER_DRIVER_HPP_
