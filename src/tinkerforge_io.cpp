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

// BrickletIo implementation on top of the vendored TinkerForge C bindings.

#include <memory>
#include <string>
#include <utility>

#include "cmeresearch_stepper_driver/bricklet_io.hpp"
#include "tinkerforge/bricklet_silent_stepper_v2.h"
#include "tinkerforge/ip_connection.h"

namespace cmeresearch_stepper_driver
{

namespace
{

// Callback signatures differ per callback id; the bindings take a generic
// function pointer and cast it back internally.
template<typename Fn>
void (* as_callback(Fn * fn))(void)
{
  return reinterpret_cast<void (*)(void)>(fn);
}

// Setters used once per (re)configuration. With a response requested, the
// bricklet's answer (e.g. E_INVALID_PARAMETER, E_TIMEOUT) is returned instead
// of the call being fire-and-forget. The hot path (drive / max velocity) stays
// unacknowledged.
constexpr uint8_t kAcknowledgedSetters[] = {
  SILENT_STEPPER_V2_FUNCTION_SET_STEP_CONFIGURATION,
  SILENT_STEPPER_V2_FUNCTION_SET_MOTOR_CURRENT,
  SILENT_STEPPER_V2_FUNCTION_SET_SPEED_RAMPING,
  SILENT_STEPPER_V2_FUNCTION_SET_BASIC_CONFIGURATION,
  SILENT_STEPPER_V2_FUNCTION_SET_STEALTH_CONFIGURATION,
  SILENT_STEPPER_V2_FUNCTION_SET_ENABLED,
};

class TinkerforgeIo : public BrickletIo
{
public:
  TinkerforgeIo()
  {
    ipcon_create(&ipcon_);
    ipcon_register_callback(&ipcon_, IPCON_CALLBACK_CONNECTED, as_callback(&on_connected), this);
    ipcon_register_callback(
      &ipcon_, IPCON_CALLBACK_DISCONNECTED, as_callback(&on_disconnected), this);
    ipcon_register_callback(&ipcon_, IPCON_CALLBACK_ENUMERATE, as_callback(&on_enumerate), this);
  }

  ~TinkerforgeIo() override
  {
    ipcon_disconnect(&ipcon_);
    destroy_device();
    ipcon_destroy(&ipcon_);
  }

  void set_callbacks(BrickletCallbacks callbacks) override {callbacks_ = std::move(callbacks);}

  int connect(const std::string & host, uint16_t port) override
  {
    return ipcon_connect(&ipcon_, host.c_str(), port);
  }

  void set_auto_reconnect(bool enabled) override {ipcon_set_auto_reconnect(&ipcon_, enabled);}

  int enumerate() override {return ipcon_enumerate(&ipcon_);}

  void disconnect() override {ipcon_disconnect(&ipcon_);}

  void create_device(const std::string & uid) override
  {
    destroy_device();
    silent_stepper_v2_create(&device_, uid.c_str(), &ipcon_);
    device_created_ = true;
    for (const uint8_t fn : kAcknowledgedSetters) {
      silent_stepper_v2_set_response_expected(&device_, fn, true);
    }
    silent_stepper_v2_register_callback(
      &device_, SILENT_STEPPER_V2_CALLBACK_ALL_DATA, as_callback(&on_all_data), this);
    silent_stepper_v2_register_callback(
      &device_, SILENT_STEPPER_V2_CALLBACK_NEW_STATE, as_callback(&on_new_state), this);
  }

  void destroy_device() override
  {
    if (device_created_) {
      silent_stepper_v2_destroy(&device_);
      device_created_ = false;
    }
  }

  int configure(const StepperConfig & c, uint8_t step_resolution_register) override
  {
    int e = silent_stepper_v2_set_step_configuration(
      &device_, step_resolution_register, c.interpolation);
    if (e == E_OK) {e = silent_stepper_v2_set_motor_current(&device_, c.motor_run_current);}
    if (e == E_OK) {e = silent_stepper_v2_set_max_velocity(&device_, 0);}
    if (e == E_OK) {
      e = silent_stepper_v2_set_speed_ramping(&device_, c.acceleration, c.deceleration);
    }
    if (e == E_OK) {
      e = silent_stepper_v2_set_basic_configuration(
        &device_, c.standstill_current, c.motor_run_current, c.standstill_delay_time,
        c.power_down_time, c.stealth_threshold, c.coolstep_threshold, c.classic_threshold,
        c.high_velocity_chopper_mode);
    }
    if (e == E_OK) {
      // Only freewheel_mode deviates from the firmware defaults (stealth on,
      // amplitude 128, gradient 4, autoscale on, force_symmetric off). It
      // governs the standstill behaviour when standstill_current == 0.
      e = silent_stepper_v2_set_stealth_configuration(
        &device_, true, 128, 4, true, false, static_cast<uint8_t>(c.freewheel_mode));
    }
    if (e == E_OK) {e = silent_stepper_v2_set_all_callback_configuration(&device_, 100);}
    if (e == E_OK) {e = silent_stepper_v2_set_enabled(&device_, true);}
    return e;
  }

  int set_enabled(bool enabled) override {return silent_stepper_v2_set_enabled(&device_, enabled);}

  int drive(int direction) override
  {
    return direction > 0 ? silent_stepper_v2_drive_forward(&device_) :
           silent_stepper_v2_drive_backward(&device_);
  }

  int set_max_velocity(uint16_t v) override {return silent_stepper_v2_set_max_velocity(&device_, v);}

  int stop() override {return silent_stepper_v2_stop(&device_);}

private:
  static void on_connected(uint8_t reason, void * user_data)
  {
    auto * self = static_cast<TinkerforgeIo *>(user_data);
    if (self->callbacks_.on_connected) {
      self->callbacks_.on_connected(reason == IPCON_CONNECT_REASON_AUTO_RECONNECT);
    }
  }

  static void on_disconnected(uint8_t /*reason*/, void * user_data)
  {
    auto * self = static_cast<TinkerforgeIo *>(user_data);
    if (self->callbacks_.on_disconnected) {
      self->callbacks_.on_disconnected();
    }
  }

  static void on_enumerate(
    const char * uid, const char * /*connected_uid*/, char position,
    uint8_t /*hardware_version*/[3], uint8_t firmware_version[3],
    uint16_t device_identifier, uint8_t enumeration_type, void * user_data)
  {
    auto * self = static_cast<TinkerforgeIo *>(user_data);
    if (!self->callbacks_.on_enumerate) {
      return;
    }
    EnumerateInfo info;
    info.uid = uid;
    info.position = position;
    info.firmware = {firmware_version[0], firmware_version[1], firmware_version[2]};
    info.is_silent_stepper_v2 = device_identifier == SILENT_STEPPER_V2_DEVICE_IDENTIFIER;
    info.type = enumeration_type == IPCON_ENUMERATION_TYPE_DISCONNECTED ?
      EnumerateInfo::Type::Disconnected :
      enumeration_type == IPCON_ENUMERATION_TYPE_CONNECTED ?
      EnumerateInfo::Type::Connected : EnumerateInfo::Type::Available;
    self->callbacks_.on_enumerate(info);
  }

  static void on_all_data(
    uint16_t velocity, int32_t position, int32_t remaining_steps, uint16_t input_voltage,
    uint16_t current_consumption, void * user_data)
  {
    auto * self = static_cast<TinkerforgeIo *>(user_data);
    if (self->callbacks_.on_all_data) {
      self->callbacks_.on_all_data(
        velocity, position, remaining_steps, input_voltage, current_consumption);
    }
  }

  static void on_new_state(uint8_t state_new, uint8_t state_previous, void * user_data)
  {
    auto * self = static_cast<TinkerforgeIo *>(user_data);
    if (self->callbacks_.on_new_state) {
      self->callbacks_.on_new_state(state_new, state_previous);
    }
  }

  IPConnection ipcon_;
  SilentStepperV2 device_;
  bool device_created_ = false;
  BrickletCallbacks callbacks_;
};

}  // namespace

std::unique_ptr<BrickletIo> make_tinkerforge_io()
{
  return std::make_unique<TinkerforgeIo>();
}

}  // namespace cmeresearch_stepper_driver
