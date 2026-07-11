#!/usr/bin/env python3
"""
Pick-and-place node using MoveGroup action client directly.

Sequences through SRDF named states, controls jaw via /teensy_jaw_target.
Triggered by /run_sequence service (std_srvs/Trigger).

Sequence: home -> pregrasp -> grasp(close jaw) -> lift -> preplace ->
          place(open jaw) -> prehome -> home
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    RobotState,
)
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

# Named state joint values from SRDF (arm4 group)
# Order: [continuous, revolute1, revolute2, revolute3]
NAMED_STATES = {
    "home":     [0.0,     0.0,       0.0,      0.0],
    "pregrasp": [-1.4136, -0.1633,   -0.4252,  -2.1694],
    "grasp":    [-1.4238, -0.418692, -0.5429,  -2.0000],
    "lift":     [-1.4232, -0.2490,   -0.5429,  -2.2800],
    "preplace": [1.5885,  -0.2592,   -0.5488,  -2.1745],
    "place":    [1.5882,  -0.5151,   -0.5488,  -2.1745],
    "prehome":  [1.5882,  -0.2592,   -0.5488,  -2.2745],
}

JOINT_NAMES = ["continuous", "revolute1", "revolute2", "revolute3"]

SEQUENCE = ["home", "pregrasp", "grasp", "lift", "preplace", "place", "prehome", "home"]

JAW_OPEN = 1.0
JAW_CLOSE = 0.0


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node")

        cbg = ReentrantCallbackGroup()

        self._move_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=cbg
        )

        self._jaw_pub = self.create_publisher(Float64, "/teensy_jaw_target", 10)

        self._srv = self.create_service(
            Trigger, "/run_sequence", self._on_trigger, callback_group=cbg
        )

        self._running = False
        self._lock = threading.Lock()

        self.get_logger().info("PickPlaceNode ready. Call /run_sequence to start.")

    def _on_trigger(self, request, response):
        with self._lock:
            if self._running:
                response.success = False
                response.message = "Already running"
                return response
            self._running = True

        try:
            ok = self._run_sequence()
            response.success = ok
            response.message = "Done" if ok else "Failed"
        except Exception as e:
            self.get_logger().error(f"Sequence error: {e}")
            response.success = False
            response.message = str(e)
        finally:
            with self._lock:
                self._running = False

        return response

    def _run_sequence(self) -> bool:
        self.get_logger().info("Waiting for /move_action action server...")
        if not self._move_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/move_action action server not available")
            return False

        self.get_logger().info("Starting pick-and-place sequence")

        # Start with jaw open
        self._set_jaw(JAW_OPEN, "OPEN")

        for state_name in SEQUENCE:
            self.get_logger().info(f"=== Moving to: {state_name} ===")

            if not self._move_to_named(state_name):
                self.get_logger().error(f"Failed at state: {state_name}")
                return False

            # Jaw actions at specific states
            if state_name == "grasp":
                self._set_jaw(JAW_CLOSE, "CLOSE")
                rclpy.spin_once(self, timeout_sec=0.3)
            elif state_name == "lift":
                # Re-tighten after lift
                self._set_jaw(JAW_CLOSE, "RE-TIGHTEN")
            elif state_name == "preplace":
                # Re-tighten before place
                self._set_jaw(JAW_CLOSE, "RE-TIGHTEN")
            elif state_name == "place":
                self._set_jaw(JAW_OPEN, "OPEN")
            else:
                self._set_jaw(JAW_OPEN, "OPEN")

        self.get_logger().info("Sequence complete")
        return True

    def _move_to_named(self, state_name: str) -> bool:
        if state_name not in NAMED_STATES:
            self.get_logger().error(f"Unknown state: {state_name}")
            return False

        joint_values = NAMED_STATES[state_name]

        # Build MoveGroup goal
        goal = MoveGroup.Goal()

        # MotionPlanRequest
        req = MotionPlanRequest()
        req.group_name = "arm4"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        # Set start state to current
        req.start_state = RobotState()
        req.start_state.is_diff = True

        # Joint constraints for the goal
        constraints = Constraints()
        for jn, jv in zip(JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = jv
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)

        goal.request = req

        # Planning options
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # Send goal
        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for state: {state_name}")
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f"No result for state: {state_name}")
            return False

        error_code = result.result.error_code.val
        if error_code != 1:  # SUCCESS = 1
            self.get_logger().error(
                f"MoveGroup failed for '{state_name}': error_code={error_code}"
            )
            return False

        self.get_logger().info(f"Reached: {state_name}")
        return True

    def _set_jaw(self, value: float, label: str):
        msg = Float64()
        msg.data = value
        # Burst publish for reliability
        for _ in range(4):
            self._jaw_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.06)
        self.get_logger().info(f"Jaw {label} -> {value:.1f}")
        rclpy.spin_once(self, timeout_sec=0.25)


def main():
    rclpy.init()
    node = PickPlaceNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
