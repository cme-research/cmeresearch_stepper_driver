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

// Configuration / feedback types and configuration validation. Free of ROS and
// TinkerForge dependencies so they can be unit tested directly.

#ifndef CMERESEARCH_STEPPER_DRIVER__STEPPER_CONFIG_HPP_
#define CMERESEARCH_STEPPER_DRIVER__STEPPER_CONFIG_HPP_

#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

#include "cmeresearch_stepper_driver/stepper_math.hpp"

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

  /// Feedback older than this is treated as lost (velocity reported as 0) [s].
  /// The bricklet streams data every 100 ms.
  double feedback_timeout_s = 0.5;
  /// Wait between attempts to reach brickd while it is not (yet) running [ms].
  int connect_retry_ms = 1000;
  /// Time given to the ramp-down before the driver stage is cut on shutdown [ms].
  int shutdown_ramp_ms = 400;
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
  /// False while the data is not live (not connected / not configured / stale).
  bool valid = false;
};

/// Result of validate(): errors make the configuration unusable, warnings are
/// accepted but logged.
struct ConfigCheck
{
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  bool ok() const {return errors.empty();}
};

namespace detail
{
inline void require_range(
  ConfigCheck & check, const char * name, long value, long min, long max)
{
  if (value < min || value > max) {
    check.errors.push_back(
      std::string(name) + " must be in [" + std::to_string(min) + ", " +
      std::to_string(max) + "] (got " + std::to_string(value) + ")");
  }
}
}  // namespace detail

/// Check a configuration against the limits of the TinkerForge API (most
/// values are transmitted as uint16 and would otherwise be truncated silently).
inline ConfigCheck validate(const StepperConfig & c)
{
  ConfigCheck check;
  using detail::require_range;
  constexpr long kU16 = 65535;

  require_range(check, "bricklet_port", c.port, 1, kU16);
  const char pos = static_cast<char>(std::tolower(static_cast<unsigned char>(c.brick_position)));
  if (pos < 'a' || pos > 'h') {
    check.errors.push_back(
      std::string("brick_position must be a bricklet port letter a..h (got '") +
      c.brick_position + "')");
  }
  if (!step_resolution_to_register(c.microsteps)) {
    check.errors.push_back(
      "step_resolution must be one of 1, 2, 4, 8, 16, 32, 64, 128, 256 (got " +
      std::to_string(c.microsteps) + ")");
  }
  require_range(check, "steps_per_revolution", c.steps_per_revolution, 1, kU16);
  require_range(check, "max_step_vel", c.max_step_vel, 1, kU16);
  require_range(check, "acceleration", c.acceleration, 0, kU16);
  require_range(check, "deceleration", c.deceleration, 0, kU16);
  require_range(check, "motor_run_current", c.motor_run_current, 1, kU16);
  // The bricklet caps the standstill current at the motor current.
  require_range(check, "standstill_current", c.standstill_current, 0, c.motor_run_current);
  require_range(check, "standstill_delay_time", c.standstill_delay_time, 0, kU16);
  require_range(check, "power_down_time", c.power_down_time, 0, kU16);
  require_range(check, "stealth_threshold", c.stealth_threshold, 0, kU16);
  require_range(check, "coolstep_threshold", c.coolstep_threshold, 0, kU16);
  require_range(check, "classic_threshold", c.classic_threshold, 0, kU16);
  require_range(check, "freewheel_mode", c.freewheel_mode, 0, 3);
  require_range(check, "under_voltage_mv", c.under_voltage_mv, 0, kU16);
  require_range(check, "under_voltage_hysteresis_mv", c.under_voltage_hysteresis_mv, 0, kU16);
  if (!(c.feedback_timeout_s > 0.0)) {
    check.errors.push_back("feedback_timeout must be > 0");
  }
  if (c.connect_retry_ms <= 0) {
    check.errors.push_back("connect_retry_ms must be > 0");
  }

  if (!(c.stealth_threshold < c.coolstep_threshold && c.coolstep_threshold < c.classic_threshold)) {
    check.warnings.push_back(
      "chopper thresholds should satisfy stealth < coolstep < classic (got " +
      std::to_string(c.stealth_threshold) + " / " + std::to_string(c.coolstep_threshold) +
      " / " + std::to_string(c.classic_threshold) + ")");
  }
  return check;
}

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__STEPPER_CONFIG_HPP_
