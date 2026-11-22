# cmeresearch_stepper_driver

ROS 2 (Jazzy) driver for the [TinkerForge Silent Stepper Bricklet 2.0](https://www.tinkerforge.com/en/doc/Hardware/Bricklets/Silent_Stepper_V2.html).
One `stepper_driver_node` runs per wheel; it talks to `brickd` over TCP and drives
the motor in continuous-drive mode (velocity control with the bricklet's own
acceleration / deceleration ramps).

```
cmexa_base (ros2_control) ──TinkerStepperCommand──▶ stepper_driver_node ──TCP──▶ brickd ──▶ bricklet
                         ◀─TinkerStepperFeedback──                      (one node per wheel)
```

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| `drive_input` (sub) | `cmeresearch_msgs/TinkerStepperCommand` | Wheel velocity `velocity` [rad/s] |
| `drive_output` (pub, 10 Hz) | `cmeresearch_msgs/TinkerStepperFeedback` | See below |
| `state` (pub, latched) | `std_msgs/String` | `initialized`, `error`, `low_voltage`, `idle` |

Feedback fields:

- `current_velocity` — **signed microsteps/s on the motor shaft**, in the command
  direction convention (`mirror_direction` already undone). The bricklet only
  reports a magnitude; the sign is taken from the direction the step counter
  moves, so it stays correct while the motor decelerates through a reversal.
  `cmexa_base` converts this to wheel rad/s.
- `current_position`, `remaining_steps` — raw bricklet step counters (motor frame).
- `input_voltage` [mV], `current_consumption` [mA].
- `header.frame_id` — `wheel_name`.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bricklet_host` / `bricklet_port` | `localhost` / `4223` | brickd address |
| `brick_position` | `a` | Bricklet port letter on the brick (`a`–`h`) |
| `step_resolution` | `8` | Microsteps per full step: 1, 2, 4, …, 256 |
| `interpolation` | `true` | 256-microstep interpolation |
| `steps_per_revolution` | `200` | Full steps per motor revolution |
| `gear_ratio` | `1.0` | Motor revolutions per wheel revolution (> 0) |
| `mirror_direction` | `false` | Invert for motors mounted the other way round |
| `max_step_vel` | `1000` | Velocity cap [microsteps/s] |
| `acceleration` / `deceleration` | `1000` | Ramps [steps/s²] |
| `wheel_name` | `wheel` | Used in logs and as feedback `frame_id` |
| `hw_simulation` | `false` | No hardware: feedback echoes the commanded velocity |
| `standstill_current`, `motor_run_current` | `200`, `800` | [mA] |
| `standstill_delay_time`, `power_down_time` | `300`, `5000` | [ms] |
| `stealth_threshold`, `coolstep_threshold`, `classic_threshold` | `400`, `500`, `600` | Chopper thresholds |
| `high_velocity_chopper_mode` | `false` | |
| `freewheel_mode` | `0` | Standstill behaviour when `standstill_current` is 0: 0 normal, 1 freewheel, 2/3 coil short LS/HS |
| `under_voltage_mv` | `17000` | Supply level that triggers `low_voltage` |
| `under_voltage_hysteresis_mv` | `500` | `low_voltage` clears at threshold + hysteresis |

Invalid `step_resolution`, `brick_position`, `gear_ratio` or `freewheel_mode`
values make the node exit with an error instead of silently misbehaving.

## Runtime behaviour

- Callbacks are registered before connecting, so the bricklet is always enumerated.
  brickd auto-reconnect is enabled; after a reconnect or a bricklet reset the
  bricklet is fully re-configured.
- After the supply recovers from under-voltage, the driver stage is disabled and
  re-enabled on the next drive command, and `idle` is published on `state`.
- A zero command ramps down without issuing a direction change.
- On shutdown the motor is stopped and the driver stage disabled.

## Build & test

```bash
colcon build --packages-select cmeresearch_stepper_driver
colcon test --packages-select cmeresearch_stepper_driver && colcon test-result --verbose
ros2 launch cmeresearch_stepper_driver tinkerforge_drivers.launch.py   # single-wheel example
```

The TinkerForge C bindings (`ip_connection`, `bricklet_silent_stepper_v2`) are
vendored under `src/tinkerforge` / `include/tinkerforge` (CC0, see `tfextern/license.txt`).
