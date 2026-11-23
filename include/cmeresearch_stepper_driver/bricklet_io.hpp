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

// Thin seam between StepperDriver and the TinkerForge C bindings, so the
// driver logic (reconnects, watchdogs, configuration errors) can be tested
// with a fake. The real implementation is tinkerforge_io.cpp.

#ifndef CMERESEARCH_STEPPER_DRIVER__BRICKLET_IO_HPP_
#define CMERESEARCH_STEPPER_DRIVER__BRICKLET_IO_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "cmeresearch_stepper_driver/stepper_config.hpp"

namespace cmeresearch_stepper_driver
{

/// One entry of a brickd enumeration.
struct EnumerateInfo
{
  enum class Type { Available, Connected, Disconnected };

  std::string uid;
  char position = '?';
  std::array<uint8_t, 3> firmware{};
  bool is_silent_stepper_v2 = false;
  Type type = Type::Available;
};

/// Callbacks from the bindings. They run on the bindings' own callback thread.
struct BrickletCallbacks
{
  std::function<void(bool auto_reconnect)> on_connected;
  std::function<void()> on_disconnected;
  std::function<void(const EnumerateInfo &)> on_enumerate;
  std::function<void(
      uint16_t velocity, int32_t position, int32_t remaining_steps,
      uint16_t input_voltage_mv, uint16_t current_consumption_ma)> on_all_data;
  std::function<void(uint8_t state_new, uint8_t state_previous)> on_new_state;
  /// A move started with set_steps / set_target_position ended at `position`.
  std::function<void(int32_t position)> on_position_reached;
};

/// Operations StepperDriver needs from brickd / one SilentStepper V2 bricklet.
/// All int results are TinkerForge E_* codes (0 = E_OK, negative = error).
class BrickletIo
{
public:
  virtual ~BrickletIo() = default;

  /// Must be called before connect().
  virtual void set_callbacks(BrickletCallbacks callbacks) = 0;

  // brickd connection
  virtual int connect(const std::string & host, uint16_t port) = 0;
  virtual void set_auto_reconnect(bool enabled) = 0;
  virtual int enumerate() = 0;
  /// Disconnects and stops the bindings' threads: no callback runs afterwards.
  virtual void disconnect() = 0;

  // bricklet
  /// Create the device handle for `uid` and subscribe to its data callbacks.
  virtual void create_device(const std::string & uid) = 0;
  virtual void destroy_device() = 0;
  /// Apply the full configuration (with acknowledgement) and enable the driver
  /// stage. Returns the first error.
  virtual int configure(const StepperConfig & config, uint8_t step_resolution_register) = 0;
  virtual int set_enabled(bool enabled) = 0;
  /// +1 drive forward, -1 drive backward.
  virtual int drive(int direction) = 0;
  virtual int set_max_velocity(uint16_t microsteps_per_s) = 0;
  virtual int stop() = 0;

  // position control (motor frame, microsteps)
  /// Move by `steps` from the current position.
  virtual int set_steps(int32_t steps) = 0;
  /// Move to the absolute counter position `position`.
  virtual int set_target_position(int32_t position) = 0;
  /// Redefine the current counter position (referencing / zeroing).
  virtual int set_current_position(int32_t position) = 0;
};

/// BrickletIo backed by the vendored TinkerForge C bindings.
std::unique_ptr<BrickletIo> make_tinkerforge_io();

}  // namespace cmeresearch_stepper_driver

#endif  // CMERESEARCH_STEPPER_DRIVER__BRICKLET_IO_HPP_
