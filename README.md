# cmeresearch_stepper_driver

ROS 2 (Jazzy) driver for the [TinkerForge Silent Stepper Bricklet 2.0](https://www.tinkerforge.com/en/doc/Hardware/Bricklets/Silent_Stepper_V2.html).
One `stepper_driver_node` runs per wheel; it talks to `brickd` over TCP and drives
the motor in continuous-drive mode (velocity control with the bricklet's own
acceleration / deceleration ramps).

```
twist_mux ─▶ mecanum controller ─▶ cmexa_base (ros2_control)
                                       │ TinkerStepperCommand  ▲ TinkerStepperFeedback
                                       ▼ (wheel rad/s)         │ (signed microsteps/s)
                              stepper_driver_node ×4 ──TCP──▶ brickd ──▶ bricklet ×4
```

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| `drive_input` (sub) | `cmeresearch_msgs/TinkerStepperCommand` | Wheel velocity `velocity` [rad/s]; `position` is ignored |
| `drive_output` (pub, 10 Hz) | `cmeresearch_msgs/TinkerStepperFeedback` | See below |
| `state` (pub, latched) | `std_msgs/String` | `initialized`, `error`, `low_voltage`, `idle` |

Feedback fields:

- `current_velocity` — **signed microsteps/s on the motor shaft**, in the command
  direction convention (`mirror_direction` already undone). The bricklet only
  reports a magnitude; the sign is taken from the direction the step counter
  moves, so it stays correct while the motor decelerates through a reversal.
  `cmexa_base` converts this to wheel rad/s:
  `wheel_rad_s = current_velocity × 2π / (steps_per_revolution × step_resolution × gear_ratio)`.
- `current_position`, `remaining_steps` — raw bricklet step counters (motor frame, not mirrored).
- `input_voltage` [mV], `current_consumption` [mA].
- `header.frame_id` — `wheel_name`.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bricklet_host` / `bricklet_port` | `localhost` / `4223` | brickd address |
| `brick_position` | `a` | Bricklet port letter on the brick (`a`–`h`, case-insensitive) |
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

### Must match `cmexa_base`

`cmexa_base` converts the feedback back to wheel velocity and normalises wheel
commands using its own copy of the drive geometry (ros2_control hardware
parameters in `cmeresearch_description`, e.g. `urdf/cmexaiii/robot_properties.urdf.xacro`).
These values must be identical on both sides, otherwise odometry is scaled wrong:

| Stepper driver parameter | `cmexa_base` hardware parameter |
|--------------------------|---------------------------------|
| `steps_per_revolution` | `steps_per_revolution` |
| `step_resolution` | `step_resolution` |
| `gear_ratio` | `gear_ratio` |
| `max_step_vel` | `max_step_vel` (optional, enables wheel-speed normalisation) |

`cmeresearch_bringup`'s tests check that all wheels of one robot use the same
values; the cross-repo match is not checked automatically yet.

## Usage on the robots

The robot launch files in `cmeresearch_bringup` (e.g. `cmexaiii_hardware.launch.py`,
`cmexa_omni.launch.py`) start one node per wheel in namespace `tf_drivers`, with one
bricklet port per wheel and these remappings:

```python
Node(
    package='cmeresearch_stepper_driver',
    executable='stepper_driver_node',
    namespace='tf_drivers',
    name='cmexa_stepper_driver_left_stepper',
    remappings=[
        ('drive_input', '/cmexa_base/front_left/cmd_vel'),
        ('drive_output', '/cmexa_base/front_left/feedback'),
        ('state', '/cmexa_base/front_left/state')],
    parameters=[{'brick_position': 'a', 'wheel_name': 'front_left_wheel', ...}],
)
```

`brickd` must be reachable at `bricklet_host:bricklet_port`. On the Raspberry Pi it
runs in its own privileged container (`cmeresearch/brickd`, see the workspace
CLAUDE.md / `cmexa_install`). For a single-wheel example see
`launch/tinkerforge_drivers.launch.py`.

## Runtime behaviour

- Callbacks are registered before connecting, so the bricklet is always enumerated.
  brickd auto-reconnect is enabled; after a reconnect or a bricklet reset the
  bricklet is fully re-configured.
- On (re)configuration the velocity setpoint is 0; the first non-zero command
  cycles the driver stage off/on and starts driving.
- After the supply recovers from under-voltage, the driver stage is disabled and
  re-enabled on the next drive command, and `idle` is published on `state`.
  The node only warns on low voltage; it does not stop the motor itself.
- A zero command ramps down without issuing a direction change; non-finite
  commands are ignored.
- On shutdown the motor is stopped and the driver stage disabled.

## Troubleshooting

| Symptom | Cause |
|---------|-------|
| `Connecting to brickd at …:4223 failed (error -4)`, node exits 1 | brickd not running or wrong `bricklet_host` / `bricklet_port` |
| `step_resolution must be one of 1, 2, 4, …` / `brick_position must be …`, node exits 1 | Invalid parameter in the launch file |
| Node runs, feedback stays 0, no `Found SilentStepper V2 … at port x` log | No Silent Stepper 2.0 at that `brick_position` |
| Wheel turns the wrong way | Flip `mirror_direction` for that wheel |
| Odometry too fast / slow | Drive geometry differs from `cmexa_base` (see *Must match*) |
| `state` = `low_voltage` | Supply below `under_voltage_mv` |

## Code layout

| Path | Contents |
|------|----------|
| `src/stepper_driver_ros_wrapper.cpp` | ROS node: parameters, topics, mirroring / gear ratio, `hw_simulation` |
| `src/tinkerforge_bricklet_stepper_driver.cpp` | `StepperDriver`: brickd connection, enumeration, bricklet config, thread-safe feedback |
| `include/cmeresearch_stepper_driver/stepper_math.hpp` | Pure helpers (unit conversion, step-resolution register, direction, under-voltage hysteresis) |
| `src/tinkerforge/`, `include/tinkerforge/` | Vendored TinkerForge C bindings (`ip_connection`, `bricklet_silent_stepper_v2`; CC0, see `tfextern/license.txt`) |
| `test/` | Unit and launch tests (below) |

## Build & test

```bash
colcon build --packages-select cmeresearch_stepper_driver
colcon test --packages-select cmeresearch_stepper_driver && colcon test-result --verbose
ros2 launch cmeresearch_stepper_driver tinkerforge_drivers.launch.py   # single-wheel example
```

Tests (no hardware needed):

- `test/test_stepper_math.cpp` (gtest) — step-resolution mapping, rad/s → microsteps/s
  conversion and clamping, direction from step-counter deltas, under-voltage hysteresis.
- `test/test_node_simulation.launch.py` (launch_testing) — runs the node with
  `hw_simulation`, mirroring and gear ratio 2: command → signed feedback, clamping,
  zero and NaN commands, latched `initialized` state, and an invalid
  `step_resolution` making the node exit with code 1.

CI (`.github/workflows/build.yml`, `ros-tooling/action-ros-ci`) builds the package with
`cmeresearch_msgs` from `.ci.repos` and runs these tests on every push and PR to
`jazzy` / `jazzy_dev`. ament linters are excluded (`ctest -LE linter`,
`pytest -m "not linter"`), matching the other cme-research ROS repos. With
cme-research/cmexa_install#103 merged, the `cmexa_install` PR pipeline also runs them
against the pinned workspace.

Hardware behaviour (bricklet configuration, reconnects, under-voltage recovery) is
not covered by CI — verify on a robot after driver changes.
