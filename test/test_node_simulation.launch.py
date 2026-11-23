# Copyright (C) 2025 Christian Ehrmann — GPLv3, see LICENSE.
"""Node-level tests for stepper_driver_node, no hardware required.

* A node in hw_simulation mode (mirror_direction, gear_ratio 2) must turn
  wheel rad/s commands into signed microsteps/s feedback in the joint
  convention, clamp to max_step_vel and latch state "initialized".
* A node with an invalid step_resolution must refuse to start (exit code 1).
"""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from cmeresearch_msgs.msg import TinkerStepperCommand, TinkerStepperFeedback

STEPS_PER_REV = 200
MICROSTEPS = 8
GEAR = 2.0
MAX_STEP_VEL = 3000


def expected_steps(wheel_rad_s):
    """Signed microsteps/s the driver should report for a wheel command."""
    magnitude = min(abs(wheel_rad_s) * GEAR / (2 * math.pi) * STEPS_PER_REV * MICROSTEPS,
                    MAX_STEP_VEL)
    return math.copysign(round(magnitude), wheel_rad_s) if magnitude else 0.0


@pytest.mark.launch_test
def generate_test_description():
    sim_node = launch_ros.actions.Node(
        package='cmeresearch_stepper_driver',
        executable='stepper_driver_node',
        namespace='sim',
        parameters=[{
            'hw_simulation': True,
            'mirror_direction': True,
            'gear_ratio': GEAR,
            'steps_per_revolution': STEPS_PER_REV,
            'step_resolution': MICROSTEPS,
            'max_step_vel': MAX_STEP_VEL,
            'wheel_name': 'front_left',
        }],
        remappings=[('state', '/sim/state')],
    )
    bad_node = launch_ros.actions.Node(
        package='cmeresearch_stepper_driver',
        executable='stepper_driver_node',
        namespace='bad',
        parameters=[{'hw_simulation': True, 'step_resolution': 12}],
    )
    return (
        launch.LaunchDescription([sim_node, bad_node, launch_testing.actions.ReadyToTest()]),
        {'sim_node': sim_node, 'bad_node': bad_node},
    )


class TestSimulatedDriver(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('stepper_driver_test')
        self.feedback = []
        self.node.create_subscription(
            TinkerStepperFeedback, '/sim/drive_output', self.feedback.append, 10)
        self.cmd_pub = self.node.create_publisher(
            TinkerStepperCommand, '/sim/drive_input', 10)

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def command_and_wait(self, wheel_rad_s):
        """Publish a command until feedback reflects the expected value."""
        want = expected_steps(wheel_rad_s)
        msg = TinkerStepperCommand(velocity=float(wheel_rad_s))

        def reached():
            self.cmd_pub.publish(msg)
            return bool(self.feedback) and self.feedback[-1].current_velocity == want

        ok = self.spin_until(reached)
        got = self.feedback[-1].current_velocity if self.feedback else None
        self.assertTrue(ok, f'command {wheel_rad_s} rad/s: expected {want}, last feedback {got}')
        return self.feedback[-1]

    def test_state_latched_initialized(self):
        states = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(String, '/sim/state', lambda m: states.append(m.data), qos)
        self.assertTrue(self.spin_until(lambda: bool(states)), 'no latched state received')
        self.assertEqual(states[-1], 'initialized')

    def test_forward_command_gives_positive_feedback(self):
        fb = self.command_and_wait(1.0)   # 1 rad/s * 2 / 2pi * 1600 = 509
        self.assertEqual(fb.current_velocity, 509.0)
        self.assertEqual(fb.header.frame_id, 'front_left')

    def test_reverse_command_gives_negative_feedback(self):
        # mirror_direction must be undone: sign follows the command, not the motor.
        fb = self.command_and_wait(-1.0)
        self.assertEqual(fb.current_velocity, -509.0)

    def test_velocity_is_clamped(self):
        fb = self.command_and_wait(-100.0)
        self.assertEqual(fb.current_velocity, -float(MAX_STEP_VEL))

    def test_zero_command_stops(self):
        self.command_and_wait(2.0)
        fb = self.command_and_wait(0.0)
        self.assertEqual(fb.current_velocity, 0.0)

    def test_non_finite_command_is_ignored(self):
        self.command_and_wait(1.0)
        self.cmd_pub.publish(TinkerStepperCommand(velocity=float('nan')))
        n = len(self.feedback)
        # Stay well inside the 0.5 s command timeout.
        self.spin_until(lambda: len(self.feedback) >= n + 2)
        self.assertEqual(self.feedback[-1].current_velocity, 509.0)

    def test_wheel_stops_when_commands_stop(self):
        """Command watchdog: no command for 0.5 s -> standstill."""
        self.command_and_wait(1.0)
        t_last_command = time.monotonic()
        stopped = self.spin_until(
            lambda: bool(self.feedback) and self.feedback[-1].current_velocity == 0.0,
            timeout=3.0)
        elapsed = time.monotonic() - t_last_command
        self.assertTrue(stopped, 'wheel kept turning without commands')
        self.assertGreaterEqual(elapsed, 0.4, 'stopped before the timeout')
        self.assertLess(elapsed, 1.0, 'command_timeout 0.5 s not honoured')

    def test_position_is_integrated_in_simulation(self):
        self.command_and_wait(1.0)
        p0 = self.feedback[-1].current_position
        self.command_and_wait(1.0)
        n = len(self.feedback)
        self.spin_until(lambda: len(self.feedback) >= n + 2)
        self.assertGreater(self.feedback[-1].current_position, p0)

    def test_parameters_are_read_only(self):
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        client = self.node.create_client(SetParameters, '/sim/stepper_bricklet/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        req = SetParameters.Request(
            parameters=[Parameter('max_step_vel', Parameter.Type.INTEGER, 100).to_parameter_msg()])
        future = client.call_async(req)
        self.assertTrue(self.spin_until(future.done))
        self.assertFalse(future.result().results[0].successful,
                         'runtime parameter change must be rejected, it would have no effect')


@launch_testing.post_shutdown_test()
class TestExitCodes(unittest.TestCase):

    def test_invalid_step_resolution_refused(self, proc_info, bad_node):
        launch_testing.asserts.assertExitCodes(proc_info, [1], process=bad_node)

    def test_sim_node_exits_cleanly(self, proc_info, sim_node):
        # SIGINT from launch -> clean shutdown; -2/-15 tolerated if the signal
        # is delivered to the process before rclcpp installs its handler.
        launch_testing.asserts.assertExitCodes(proc_info, [0, -2, -15], process=sim_node)
