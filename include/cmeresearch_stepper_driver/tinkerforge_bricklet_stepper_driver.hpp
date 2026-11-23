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
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "tinkerforge/ip_connection.h"
#include "tinkerforge/bricklet_silent_stepper_v2.h"

namespace cmeresearch_stepper_driver
{

/// Static configuration of one SilentStepper V2 bricklet / wheel.
struct StepperConfig
{
  std::string host = "localhost";
  int port = 4223;
  /// Bricklet port letter on the brick ('a'..'h', case-insensitive).
  char brick_position = 'a';

  /// Microsteps per full step: 1, 2, 4, ..., 256.
  int microsteps = 8;
  bool interpolation = true;
  int steps_per_revolution = 200;
  int acceleration = 1000;    // steps/s^2
  int deceleration = 1000;    // steps/s^2
  int max_step_vel = 1000;    // microsteps/s

  // Motor current / chopper configuration (see set_basic_configuration).
  int standstill_current = 200;
  int motor_run_current = 800;
  int standstill_delay_time = 300;
  int power_down_time = 5000;
  int stealth_threshold = 400;
  int coolstep_threshold = 500;
  int classic_threshold = 600;
  bool high_velocity_chopper_mode = false;
  /// Standstill behaviour when standstill_current == 0:
  /// 0 = Normal, 1 = Freewheeling, 2 = Coil short LS, 3 = Coil short HS.
  int freewheel_mode = 0;

  /// Input voltage below which the supply is considered too low [mV].
  int under_voltage_mv = 17000;
  /// The low-voltage state clears at under_voltage_mv + this value [mV].
  int under_voltage_hysteresis_mv = 500;
};

/// Snapshot of the most recent data reported by the bricklet.
struct StepperFeedback
{
  /// Signed motor-shaft velocity [microsteps/s]. The bricklet only reports the
  /// magnitude; the sign is derived from the direction the position moves.
  int32_t velocity = 0;
  int32_t position = 0;          // microsteps, motor frame
  int32_t remaining_steps = 0;
  int32_t input_voltage_mv = 0;
  int32_t current_consumption_ma = 0;
  uint8_t state = 0;             // SILENT_STEPPER_V2_STATE_*, 0 = unknown
  uint8_t previous_state = 0;
  bool under_voltage = false;
};

enum class LogLevel { Debug, Info, Warn, Error };
using LogFn = std::function<void(LogLevel, const std::string &)>;

/// Drives one TinkerForge SilentStepper Bricklet 2.0 in continuous-drive mode.
///
/// Threading: the TinkerForge binding invokes the static cb_* callbacks on its
/// own callback thread, while the public methods are called from the ROS
/// executor. Device lifetime/configuration is guarded by device_mutex_, the
/// feedback snapshot by feedback_mutex_, and single-word flags are atomics.
class StepperDriver
{
public:
  explicit StepperDriver(StepperConfig config, LogFn log = nullptr);
  ~StepperDriver();

  StepperDriver(const StepperDriver &) = delete;
  StepperDriver & operator=(const StepperDriver &) = delete;

  /// Connect to brickd. The bricklet is configured asynchronously once it is
  /// enumerated (and again after every reconnect / bricklet reset).
  /// Returns 0 on success or the negative TinkerForge E_* error code.
  int connect();

  /// True once the matching bricklet was found and configured.
  bool is_configured() const { return configured_.load(); }

  /// Command a signed motor-shaft velocity [rad/s] (motor frame, i.e. after
  /// gear ratio and direction mirroring). 0 ramps down to standstill.
  void command_velocity(double motor_rad_s);

  /// Ramp the motor down to standstill.
  void stop();

  StepperFeedback feedback() const;

  /// Returns true exactly once after the supply recovered from under-voltage.
  bool consume_voltage_recovered() { return voltage_recovered_event_.exchange(false); }

private:
  void log(LogLevel level, const std::string & msg) const;
  void configure_device_locked(const char * uid);
  void ensure_enabled_locked();

  static void cb_connected(uint8_t reason, void * user_data);
  static void cb_enumerate(
    const char * uid, const char * connected_uid, char position,
    uint8_t hardware_version[3], uint8_t firmware_version[3],
    uint16_t device_identifier, uint8_t enumeration_type, void * user_data);
  static void cb_all_data(
    uint16_t current_velocity, int32_t current_position, int32_t remaining_steps,
    uint16_t input_voltage, uint16_t current_consumption, void * user_data);
  static void cb_new_state(uint8_t state_new, uint8_t state_previous, void * user_data);

  const StepperConfig config_;
  const uint8_t step_resolution_register_;
  LogFn log_;

  IPConnection ipcon_;
  SilentStepperV2 device_;

  // Guards device_ creation/configuration and the command path.
  mutable std::mutex device_mutex_;
  bool device_created_ = false;     // device_ holds a live handle
  std::string device_uid_;
  bool enabled_ = false;            // driver stage enabled by us since (re)configure

  std::atomic<bool> configured_{false};
  // Last direction sent to the bricklet (+1/-1); read by cb_all_data as the
  // sign fallback while the position is not changing.
  std::atomic<int> commanded_direction_{1};
  std::atomic<bool> rearm_pending_{false};          // consumed by the command path
  std::atomic<bool> voltage_recovered_event_{false};  // consumed by consume_voltage_recovered()

  mutable std::mutex feedback_mutex_;
  StepperFeedback feedback_;
  bool have_position_ = false;
};

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__TINKERFORGE_BRICKLET_STEPPER_DRIVER_HPP_
