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

// Pure, hardware-free helpers used by the stepper driver. Kept header-only and
// free of ROS / TinkerForge dependencies so they can be unit tested directly.

#ifndef CMERESEARCH_STEPPER_DRIVER__STEPPER_MATH_HPP_
#define CMERESEARCH_STEPPER_DRIVER__STEPPER_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace cmeresearch_stepper_driver
{

/// Map a microstep count (1, 2, 4, ..., 256) to the SilentStepper V2
/// STEP_RESOLUTION_* register value (1 -> 8, 2 -> 7, ..., 256 -> 0).
/// Returns std::nullopt for any value that is not a supported power of two.
inline std::optional<uint8_t> step_resolution_to_register(int microsteps)
{
  for (uint8_t reg = 0; reg <= 8; ++reg) {
    if (microsteps == (1 << (8 - reg))) {
      return reg;
    }
  }
  return std::nullopt;
}

/// Convert a motor-shaft angular speed magnitude [rad/s] into the bricklet's
/// velocity setpoint [microsteps/s], clamped to [0, max_step_vel] and to the
/// uint16_t range of the hardware register. The sign of motor_rad_s is ignored.
inline uint16_t rad_s_to_step_velocity(
  double motor_rad_s, int steps_per_revolution, int microsteps, int max_step_vel)
{
  if (!std::isfinite(motor_rad_s)) {
    return 0;
  }
  const double requested =
    std::abs(motor_rad_s) / (2.0 * M_PI) * steps_per_revolution * microsteps;
  const double cap = std::clamp(static_cast<double>(max_step_vel), 0.0, 65535.0);
  return static_cast<uint16_t>(std::lround(std::min(requested, cap)));
}

/// Direction of travel derived from two consecutive position readings
/// (+1 forward, -1 backward). If the position did not change, `fallback` is
/// returned (the motor is at or near standstill, so the sign barely matters).
inline int direction_from_position_delta(int32_t previous, int32_t current, int fallback)
{
  // Compute in 64 bit so the subtraction cannot overflow at the int32 wrap.
  const int64_t delta = static_cast<int64_t>(current) - static_cast<int64_t>(previous);
  if (delta > 0) {
    return 1;
  }
  if (delta < 0) {
    return -1;
  }
  return fallback;
}

/// Under-voltage detector with hysteresis: trips when the input voltage drops
/// below `trip_mv`, clears only once it rises to `trip_mv + hysteresis_mv` or
/// above. Returns the new latched state.
inline bool update_under_voltage(bool latched, int input_mv, int trip_mv, int hysteresis_mv)
{
  if (!latched) {
    return input_mv < trip_mv;
  }
  return input_mv < trip_mv + std::max(hysteresis_mv, 0);
}

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__STEPPER_MATH_HPP_
