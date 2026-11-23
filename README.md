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
| `state` (pub, latched, on change) | `std_msgs/String` | See [Driver states](#driver-states) |

Feedback fields:

- `current_velocity` — **signed microsteps/s on the motor shaft**, in the command
  direction convention (`mirror_direction` already undone). The bricklet only
  reports a magnitude; the sign is taken from the direction the step counter
  moves, so it stays correct while the motor decelerates through a reversal.
  `cmexa_base` converts this to wheel rad/s:
  `wheel_rad_s = current_velocity × 2π / (steps_per_revolution × step_resolution × gear_ratio)`.
  It is **0 whenever the data is not live** (not connected, bricklet not
  configured, or no data for `feedback_timeout`), so odometry never integrates
  a frozen value.
- `current_position`, `remaining_steps` — raw bricklet step counters (motor frame, not mirrored).
  In `hw_simulation` the position is integrated from the commanded velocity.
- `input_voltage` [mV], `current_consumption` [mA].
- `header.frame_id` — `wheel_name`.

## Driver states

`state` is latched and published whenever it changes:

| State | Meaning |
|-------|---------|
| `connecting` | brickd not reached yet, or the bricklet at `brick_position` not configured yet. Retried every `connect_retry_ms`. |
| `initialized` | Bricklet configured and streaming data — ready to drive. |
| `low_voltage` | Supply below `under_voltage_mv`. The node only reports it; `cmeresearch_robot_state` triggers the emergency stop. |
| `idle` | Ready again after a low-voltage phase (the driver stage is re-armed on the next command). |
| `disconnected` | Was ready, but brickd/bricklet is gone or no data for `feedback_timeout`. Reconnects automatically; then back to `initialized`. `cmeresearch_robot_state` triggers an emergency stop; after a reset it waits until the driver is `initialized` again. |
| `error` | Invalid parameters (node exits) or the bricklet rejected the configuration (retried after the next reconnect / bricklet reset). |

`cmeresearch_robot_state` treats `initialized` and `idle` as ready, so the robot
only leaves its initialising state once all four bricklets are actually configured.

## Parameters

All parameters are read once at start-up and are **read-only** — `ros2 param set`
is rejected; change them in the launch / parameter file and restart the node.
Values are validated against the bricklet API ranges; invalid values put the
node into `error` and make it exit instead of being truncated silently.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bricklet_host` / `bricklet_port` | `localhost` / `4223` | brickd address |
| `brick_position` | `a` | Bricklet port letter on the brick (`a`–`h`, case-insensitive) |
| `step_resolution` | `8` | Microsteps per full step: 1, 2, 4, …, 256 |
| `interpolation` | `true` | 256-microstep interpolation |
| `steps_per_revolution` | `200` | Full steps per motor revolution |
| `gear_ratio` | `1.0` | Motor revolutions per wheel revolution (> 0) |
| `mirror_direction` | `false` | Invert for motors mounted the other way round |
| `max_step_vel` | `1000` | Velocity cap [microsteps/s], 1–65535 |
| `acceleration` / `deceleration` | `1000` | Ramps [steps/s²], 0–65535 |
| `wheel_name` | `wheel` | Used in logs and as feedback `frame_id` |
| `hw_simulation` | `false` | No hardware: feedback echoes the commanded velocity |
| `command_timeout` | `0.5` | Stop the wheel if no velocity command arrives for this long [s]; `0` disables |
| `feedback_timeout` | `0.5` | Bricklet data older than this counts as lost [s] (data arrives every 100 ms) |
| `connect_retry_ms` | `1000` | Wait between attempts to reach brickd [ms] |
| `standstill_current`, `motor_run_current` | `200`, `800` | [mA]; `standstill_current` ≤ `motor_run_current` |
| `standstill_delay_time`, `power_down_time` | `300`, `5000` | [ms] |
| `stealth_threshold`, `coolstep_threshold`, `classic_threshold` | `400`, `500`, `600` | Chopper thresholds; should satisfy stealth < coolstep < classic (warning otherwise) |
| `high_velocity_chopper_mode` | `false` | |
| `freewheel_mode` | `0` | Standstill behaviour when `standstill_current` is 0: 0 normal, 1 freewheel, 2/3 coil short LS/HS |
| `under_voltage_mv` | `17000` | Supply level that triggers `low_voltage` |
| `under_voltage_hysteresis_mv` | `500` | `low_voltage` clears at threshold + hysteresis |

`config/cmexaiii.yaml` contains the full CMEXAIII configuration as a parameter file.

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

The robot launch files in `cmeresearch_bringup` (e.g. `cmexaiii_hardware.launch.py`)
start one node per wheel, with one bricklet port per wheel and these remappings:

```python
Node(
    package='cmeresearch_stepper_driver',
    executable='stepper_driver_node',
    name='stepper_driver_left_stepper',
    remappings=[
        ('drive_input', '/cmexaiii/front_left/cmd_vel'),
        ('drive_output', '/cmexaiii/front_left/feedback'),
        ('state', '/cmexaiii/front_left/state')],
    parameters=[{'brick_position': 'a', 'wheel_name': 'front_left_wheel', ...}],
)
```

`brickd` must be reachable at `bricklet_host:bricklet_port`. On the Raspberry Pi it
runs in its own privileged container (`cmexa_brickd`, deployed by `cmexa_install`).
The node does not need brickd to be up first: it stays in `connecting` and retries.
For a single-wheel example see `launch/tinkerforge_drivers.launch.py`.

## Runtime behaviour

- **Connection:** attempts to reach brickd repeat every `connect_retry_ms` until it
  answers; afterwards the TinkerForge bindings reconnect automatically. A lost
  connection is detected (`disconnected`), and after every reconnect or bricklet
  reset the bricklet is fully re-configured.
- **Configuration** is sent with acknowledgement; a rejected value is logged and
  reported as `error`.
- **Command watchdog:** the bricklet keeps driving at the last commanded speed
  (continuous-drive mode). If no command arrives for `command_timeout`, the node
  commands standstill and logs a warning. Commands normally arrive at the
  controller rate, so this only triggers when the controller stopped.
- **Feedback timeout:** without bricklet data for `feedback_timeout` the feedback
  velocity is 0 and the state `disconnected`.
- On (re)configuration the velocity setpoint is 0; the first non-zero command
  cycles the driver stage off/on and starts driving.
- After the supply recovers from under-voltage, the driver stage is re-armed on
  the next drive command and `idle` is published.
- A zero command ramps down without issuing a direction change; non-finite
  commands are ignored.
- On shutdown the motor is stopped and the driver stage disabled.

## Troubleshooting

| Symptom | Cause |
|---------|-------|
| `brickd at …:4223 not reachable (error -4), retrying …`, state `connecting` | brickd not running (yet) or wrong `bricklet_host` / `bricklet_port` |
| `invalid parameter: …`, state `error`, node exits 1 | Invalid value in the launch / parameter file |
| State stays `connecting`, no `Found SilentStepper V2 … at port x` log | No Silent Stepper 2.0 at that `brick_position` |
| `rejected the configuration (error …)`, state `error` | The bricklet refused a value (e.g. current above its limit) |
| `No velocity command for 0.50 s, stopping the wheel` | The controller (`cmexa_base` / `ros2_control_node`) stopped sending |
| State `disconnected` | brickd restarted / USB lost / bricklet unplugged; reconnects automatically |
| Wheel turns the wrong way | Flip `mirror_direction` for that wheel |
| Odometry too fast / slow | Drive geometry differs from `cmexa_base` (see *Must match*) |
| `state` = `low_voltage` | Supply below `under_voltage_mv` |

## Code layout

| Path | Contents |
|------|----------|
| `src/stepper_driver_ros_wrapper.cpp` | ROS node: parameters, topics, states, command watchdog, mirroring / gear ratio, `hw_simulation` |
| `src/tinkerforge_bricklet_stepper_driver.cpp` | `StepperDriver`: connection retries, enumeration, configuration, feedback / staleness, command path — no ROS, no TinkerForge |
| `src/tinkerforge_io.cpp` | `BrickletIo` implementation on the TinkerForge bindings |
| `include/cmeresearch_stepper_driver/bricklet_io.hpp` | `BrickletIo` interface (seam used by the tests) |
| `include/cmeresearch_stepper_driver/stepper_config.hpp` | `StepperConfig`, `StepperFeedback`, parameter validation |
| `include/cmeresearch_stepper_driver/stepper_math.hpp` | Pure helpers (unit conversion, step-resolution register, direction, under-voltage hysteresis, command watchdog) |
| `src/tinkerforge/`, `include/tinkerforge/` | Vendored TinkerForge C bindings (`ip_connection`, `bricklet_silent_stepper_v2`; CC0, see `tfextern/license.txt`) |
| `config/cmexaiii.yaml` | Example parameter file (CMEXAIII) |
| `test/` | Unit and launch tests (below) |

## Build & test

```bash
colcon build --packages-select cmeresearch_stepper_driver
colcon test --packages-select cmeresearch_stepper_driver && colcon test-result --verbose
ros2 launch cmeresearch_stepper_driver tinkerforge_drivers.launch.py   # single-wheel example
```

Tests (no hardware needed):

- `test/test_stepper_math.cpp` (gtest) — step-resolution mapping, rad/s → microsteps/s
  conversion and clamping, direction from step-counter deltas, under-voltage
  hysteresis, command watchdog.
- `test/test_stepper_driver.cpp` (gtest) — `StepperDriver` against a fake
  `BrickletIo`: parameter validation, connection retries, only the bricklet at
  `brick_position` is configured, rejected configuration → `error`, stale data and
  brickd disconnects report velocity 0, reconfiguration after reconnect, command
  path, under-voltage re-arm, shutdown.
- `test/test_node_simulation.launch.py` (launch_testing) — runs the node with
  `hw_simulation`, mirroring and gear ratio 2: command → signed feedback, clamping,
  zero and NaN commands, command watchdog (stop after 0.5 s), integrated position,
  read-only parameters, latched `initialized` state, and an invalid
  `step_resolution` making the node exit with code 1.

CI (`.github/workflows/build.yml`, `ros-tooling/action-ros-ci`) builds the package with
`cmeresearch_msgs` from `.ci.repos` (its `jazzy_dev` branch) and runs these tests on
every push and PR to `jazzy` / `jazzy_dev`. ament linters are excluded
(`ctest -LE linter`, `pytest -m "not linter"`), matching the other cme-research ROS
repos. The `cmexa_install` PR pipeline also runs them against the pinned workspace.

The real TinkerForge path (bricklet configuration, reconnects, under-voltage
recovery on real hardware) is not covered by CI — verify on a robot after driver
changes. See `CHANGELOG.rst` for the history.
