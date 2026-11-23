^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cmeresearch_stepper_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (unreleased)
------------------

* Position control for stand-alone actuators: ``control_mode: position``
  provides the ``move_steps`` action (``cmeresearch_msgs/action/MoveSteps``:
  relative / absolute moves in microsteps with a velocity limit, cancelable,
  feedback with remaining steps) and the ``zero_position`` service.
  Absolute moves require a reference, which is cleared on every bricklet
  (re)configuration. Moves are aborted on emergency stop (``robot_state_topic``),
  disconnect, reconfiguration or low supply voltage. Simulated in
  ``hw_simulation``.
* Bricklet position setters are acknowledged.
* Example ``config/actuator_example.yaml``.
* Velocity mode (wheels) unchanged; ``control_mode`` defaults to ``velocity``.

0.2.0
-----

Safety / robustness

* Command watchdog: a wheel is stopped when no velocity command arrives for
  ``command_timeout`` (default 0.5 s). Previously the last command kept the
  wheel turning if the controller died (continuous-drive mode).
* Lost data is no longer reported as motion: when brickd disconnects or the
  bricklet stops streaming for ``feedback_timeout`` (0.5 s), the feedback
  velocity is 0 and ``state`` becomes ``disconnected``. Previously the last
  velocity stayed frozen and odometry kept integrating it.
* brickd not reachable at start-up is retried every ``connect_retry_ms``
  (``state`` = ``connecting``) instead of the node exiting for good.
* The bricklet configuration is acknowledged; a rejected value sets
  ``state`` = ``error`` instead of being ignored.
* All parameters are validated against the bricklet API ranges (values used to
  be truncated to uint16 silently) and declared read-only, so ``ros2 param
  set`` fails visibly instead of having no effect.
* ``state`` is only ``initialized`` once the bricklet is configured, not when
  brickd merely accepted the connection.
* Signed feedback velocity (sign from the step counter); ``mirror_direction``
  undone in feedback.
* Fixed: initial enumeration could be missed, uninitialised state fields,
  unused ``step_resolution``/``interpolation``, use of the device before it
  existed, data races with the bindings' callback thread.

Features

* ``hw_simulation`` integrates the position and never touches hardware.
* New parameters ``command_timeout``, ``feedback_timeout``,
  ``connect_retry_ms``, ``under_voltage_mv``, ``under_voltage_hysteresis_mv``,
  ``freewheel_mode``.
* Example parameter file ``config/cmexaiii.yaml``.

Internal

* TinkerForge access behind ``BrickletIo``; driver logic unit-tested with a fake
  (``test/test_stepper_driver.cpp``), node tested via launch_testing in
  ``hw_simulation``; tests run in CI.
* Only the two TinkerForge binding units that are used are vendored; removed
  the unused bindings/examples and the outdated ``docker/ros2`` image.
* CMake uses ``target_link_libraries`` instead of ``ament_target_dependencies``.
* ``package.xml``: SPDX licenses (``GPL-3.0-or-later``, ``CC0-1.0`` for the
  bindings), repository URL.

0.1.0
-----

* Initial ROS 2 driver for the TinkerForge Silent Stepper Bricklet 2.0.
