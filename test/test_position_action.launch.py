# Copyright (C) 2025 Christian Ehrmann — GPLv3, see LICENSE.
"""control_mode "position": MoveSteps action + zero_position service (hw_simulation)."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from cmeresearch_msgs.action import MoveSteps
from cmeresearch_msgs.msg import RobotState

NS = '/act'
MAX_STEP_VEL = 4000
ROBOT_STATE = '/act_test/robot_state'


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='cmeresearch_stepper_driver',
        executable='stepper_driver_node',
        namespace='act',
        parameters=[{
            'hw_simulation': True,
            'control_mode': 'position',
            'max_step_vel': MAX_STEP_VEL,
            'wheel_name': 'lift',
            'robot_state_topic': ROBOT_STATE,
        }],
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'node': node}


class TestPositionAction(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('position_action_test')
        cls.client = ActionClient(cls.node, MoveSteps, f'{NS}/move_steps')
        cls.zero = cls.node.create_client(Trigger, f'{NS}/zero_position')
        cls.robot_state = cls.node.create_publisher(
            RobotState, ROBOT_STATE,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        assert cls.client.wait_for_server(timeout_sec=15.0), 'move_steps server missing'
        assert cls.zero.wait_for_service(timeout_sec=15.0), 'zero_position service missing'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def send(self, mode, steps, max_velocity=0, feedback=None):
        goal = MoveSteps.Goal(mode=mode, steps=steps, max_velocity=max_velocity)
        future = self.client.send_goal_async(goal, feedback_callback=feedback)
        self.assertTrue(self.spin_until(future.done))
        return future.result()

    def result(self, handle, timeout=15.0):
        future = handle.get_result_async()
        self.assertTrue(self.spin_until(future.done, timeout), 'move did not finish')
        return future.result()

    def call_zero(self):
        future = self.zero.call_async(Trigger.Request())
        self.assertTrue(self.spin_until(future.done))
        return future.result()

    def set_robot_state(self, state):
        self.robot_state.publish(RobotState(state=state))
        time.sleep(0.3)

    # Tests run in name order and share the simulated axis.

    def test_1_relative_move_succeeds_with_feedback(self):
        feedback = []
        t0 = time.monotonic()
        handle = self.send(MoveSteps.Goal.RELATIVE, 2000, 4000,
                           feedback=lambda f: feedback.append(f.feedback))
        self.assertTrue(handle.accepted)
        res = self.result(handle)
        elapsed = time.monotonic() - t0
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED, res.result.message)
        self.assertTrue(res.result.success)
        self.assertEqual(res.result.final_position, 2000)
        self.assertGreater(len(feedback), 3)
        self.assertGreater(feedback[0].remaining_steps, feedback[-1].remaining_steps)
        self.assertGreater(elapsed, 0.3, '2000 microsteps at 4000/s take ~0.5 s')

    def test_2_absolute_move_needs_reference(self):
        handle = self.send(MoveSteps.Goal.ABSOLUTE, 0)
        res = self.result(handle)
        self.assertEqual(res.status, GoalStatus.STATUS_ABORTED)
        self.assertIn('zero_position', res.result.message)

    def test_3_zero_then_absolute_move(self):
        self.assertTrue(self.call_zero().success)
        res = self.result(self.send(MoveSteps.Goal.ABSOLUTE, -1000))
        self.assertEqual(res.status, GoalStatus.STATUS_SUCCEEDED, res.result.message)
        self.assertEqual(res.result.final_position, -1000)

    def test_4_second_goal_is_rejected_while_moving(self):
        first = self.send(MoveSteps.Goal.RELATIVE, 4000, 4000)
        self.assertTrue(first.accepted)
        second = self.send(MoveSteps.Goal.RELATIVE, 10)
        self.assertFalse(second.accepted)
        self.assertEqual(self.result(first).status, GoalStatus.STATUS_SUCCEEDED)

    def test_5_cancel_stops_the_move(self):
        handle = self.send(MoveSteps.Goal.RELATIVE, 40000, 1000)   # would take 40 s
        time.sleep(0.5)
        cancel = handle.cancel_goal_async()
        self.assertTrue(self.spin_until(cancel.done))
        res = self.result(handle)
        self.assertEqual(res.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(res.result.success)
        self.assertLess(abs(res.result.final_position - 3000), 2000,
                        'should stop shortly after the cancel')

    def test_6_emergency_stop_aborts_and_blocks_moves(self):
        handle = self.send(MoveSteps.Goal.RELATIVE, 40000, 1000)
        time.sleep(0.3)
        self.set_robot_state('emergency_stop')
        res = self.result(handle)
        self.assertEqual(res.status, GoalStatus.STATUS_ABORTED)
        self.assertIn('emergency stop', res.result.message)

        blocked = self.result(self.send(MoveSteps.Goal.RELATIVE, 10))
        self.assertEqual(blocked.status, GoalStatus.STATUS_ABORTED)

        self.set_robot_state('idle')
        ok = self.result(self.send(MoveSteps.Goal.RELATIVE, 10))
        self.assertEqual(ok.status, GoalStatus.STATUS_SUCCEEDED)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exits_cleanly(self, proc_info, node):
        launch_testing.asserts.assertExitCodes(proc_info, [0, -2, -15], process=node)
