#!/usr/bin/env python3
"""
Minimal MoveIt <-> Teensy bridge.

- Relays /teensy/joint_states -> /joint_states (sign-corrects continuous joint)
- FollowJointTrajectory action server at /arm_controller/follow_joint_trajectory
- FollowJointTrajectory action server at /gripper_controller/follow_joint_trajectory
- Extracts final waypoint, publishes to /teensy_joint_targets, waits for convergence
"""

import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


ARM_JOINTS = ["continuous", "revolute1", "revolute2"]
GRIPPER_JOINTS = ["revolute3"]

GOAL_TOLERANCES = {
    "continuous": 0.08,
    "revolute1": 0.15,
    "revolute2": 0.12,
    "revolute3": 0.35,
}

SETTLE_SEC = 0.3
CHECK_DT = 0.02
TIMEOUT_SCALING = 10.0
TIMEOUT_MARGIN = 20.0
MIN_TIMEOUT = 45.0
CONTINUOUS_LIMIT = 3.0


class TeensyBridge(Node):
    def __init__(self):
        super().__init__("teensy_bridge")
        self._pos: Dict[str, float] = {}
        self._last_js: Optional[JointState] = None

        cbg = ReentrantCallbackGroup()

        # Publishers
        self._js_pub = self.create_publisher(
            JointState, "/joint_states", qos_profile_sensor_data
        )
        self._cmd_pub = self.create_publisher(Float64MultiArray, "/teensy_joint_targets", 10)
        self._grip_pub = self.create_publisher(Float64, "/teensy_gripper_angle_target", 10)

        # Subscriber
        self.create_subscription(
            JointState,
            "/teensy/joint_states",
            self._on_teensy_js,
            qos_profile_sensor_data,
            callback_group=cbg,
        )

        # Action servers
        self._arm_as = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self._exec_arm,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cbg,
        )
        self._grip_as = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self._exec_gripper,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cbg,
        )

        self.get_logger().info("TeensyBridge ready")

    # -- Joint state relay with sign correction --
    def _on_teensy_js(self, msg: JointState):
        pos_out = list(msg.position) if msg.position else []
        vel_out = list(msg.velocity) if msg.velocity else []

        for i, name in enumerate(msg.name):
            if i < len(pos_out):
                p = float(pos_out[i])
                if name == "continuous":
                    p = -p
                    pos_out[i] = p
                    if i < len(vel_out):
                        vel_out[i] = -float(vel_out[i])
                self._pos[name] = float(pos_out[i])

        self._last_js = msg

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.name = list(msg.name)
        out.position = pos_out
        out.velocity = vel_out
        out.effort = list(msg.effort) if msg.effort else []
        self._js_pub.publish(out)

    # -- Action callbacks --
    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    # -- Arm execution --
    def _exec_arm(self, goal_handle):
        if not self._wait_for_js():
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        points = list(traj.points)

        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        # Extract final waypoint targets
        last_pt = points[-1]
        desired: Dict[str, float] = {}
        for i, jn in enumerate(names):
            if i < len(last_pt.positions):
                desired[jn] = float(last_pt.positions[i])

        # Compute timeout from trajectory duration
        total_t = float(last_pt.time_from_start.sec) + float(
            last_pt.time_from_start.nanosec
        ) * 1e-9
        timeout = max(total_t * TIMEOUT_SCALING + TIMEOUT_MARGIN, MIN_TIMEOUT)

        # Publish final target to Teensy
        cont = min(max(desired.get("continuous", self._pos.get("continuous", 0.0)),
                       -CONTINUOUS_LIMIT), CONTINUOUS_LIMIT)
        r1 = desired.get("revolute1", self._pos.get("revolute1", 0.0))
        r2 = desired.get("revolute2", self._pos.get("revolute2", 0.0))

        cmd = Float64MultiArray()
        if "revolute3" in names:
            r3 = desired.get("revolute3", self._pos.get("revolute3", 0.0))
            cmd.data = [float(cont), float(r1), float(r2), float(r3)]
        else:
            cmd.data = [float(cont), float(r1), float(r2)]
        self._cmd_pub.publish(cmd)

        # Wait for convergence
        return self._wait_converge(goal_handle, desired, timeout)

    # -- Gripper execution --
    def _exec_gripper(self, goal_handle):
        if not self._wait_for_js():
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        traj = goal_handle.request.trajectory
        points = list(traj.points)

        if not points or not points[-1].positions:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        r3 = float(points[-1].positions[0])
        m = Float64()
        m.data = r3
        self._grip_pub.publish(m)

        desired = {"revolute3": r3}
        return self._wait_converge(goal_handle, desired, 10.0)

    # -- Helpers --
    def _wait_for_js(self) -> bool:
        t_end = time.monotonic() + 5.0
        while time.monotonic() < t_end and rclpy.ok():
            if self._last_js is not None:
                return True
            time.sleep(0.02)
        return self._last_js is not None

    def _wait_converge(
        self,
        goal_handle,
        desired: Dict[str, float],
        timeout: float,
    ):
        t_start = time.monotonic()
        ok_since: Optional[float] = None

        while rclpy.ok():
            now = time.monotonic()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # Check all joints within tolerance
            all_ok = True
            for j, des in desired.items():
                act = self._pos.get(j, 0.0)
                tol = GOAL_TOLERANCES.get(j, 0.15)
                if abs(act - des) > tol:
                    all_ok = False
                    break

            if all_ok:
                if ok_since is None:
                    ok_since = now
                elif (now - ok_since) >= SETTLE_SEC:
                    goal_handle.succeed()
                    return FollowJointTrajectory.Result()
            else:
                ok_since = None

            if (now - t_start) > timeout:
                errs = {
                    j: self._pos.get(j, 0.0) - des for j, des in desired.items()
                }
                self.get_logger().error(
                    "Convergence timeout: "
                    + " ".join(f"{k}:err={v:+.3f}" for k, v in errs.items())
                )
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            time.sleep(CHECK_DT)

        goal_handle.abort()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = TeensyBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
