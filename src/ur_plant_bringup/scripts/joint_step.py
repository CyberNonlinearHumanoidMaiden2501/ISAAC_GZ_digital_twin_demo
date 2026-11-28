#!/usr/bin/env python3
# Single-joint step through the JTC action interface (tuning helper).
#
# Sends a FollowJointTrajectory goal on
# /scaled_joint_trajectory_controller/follow_joint_trajectory that moves ONE
# joint while HOLDING the other five at their current positions. The plant's
# allow_partial_joints_goal is false, so the goal always lists all six.
#
# Usage (workspace sourced):
#   python3 src/ur_plant_bringup/scripts/joint_step.py shoulder_lift_joint --delta 0.1
#   python3 src/ur_plant_bringup/scripts/joint_step.py wrist_1_joint --to 1.57
#   python3 src/ur_plant_bringup/scripts/joint_step.py elbow_joint --delta -0.1 --duration 0.5
#
# Result codes: 0 = SUCCESSFUL, -5 = GOAL_TOLERANCE_VIOLATED (the expected
# tuning signal), others = INVALID_*/PATH_TOLERANCE_VIOLATED.
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class JointStepClient(Node):
    def __init__(self):
        super().__init__("joint_step_client")
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )
        # Ground-truth joint states (the broadcaster is remapped off
        # /joint_states; the noisy /plant/joint_states arrives with the
        # sensors stage).
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_state, 10)
        self.current = None

    def _on_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.current = [pos[j] for j in JOINTS]

    def wait_for_state(self):
        self.get_logger().info("Waiting for /joint_states ...")
        while self.current is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Current positions: {self.current}")

    def send(self, joint, value, is_delta, duration):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("JTC action server not available")
            return 1
        idx = JOINTS.index(joint)
        goal_pos = list(self.current)
        goal_pos[idx] = goal_pos[idx] + value if is_delta else value

        traj = JointTrajectory()
        traj.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = goal_pos
        point.velocities = [0.0] * 6
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance.sec = 10  # generous; we measure, not race

        self.get_logger().info(
            f"Sending {joint} -> {goal_pos[idx]:.3f} rad "
            f"(slew {duration} s, others held)")
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("Goal rejected")
            return 1
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result().result
        self.get_logger().info(
            f"Result: error_code={res.error_code} '{res.error_string}'")
        return 0 if res.error_code == 0 else res.error_code


def main():
    parser = argparse.ArgumentParser(description="Single-joint JTC step")
    parser.add_argument("joint", choices=JOINTS)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--delta", type=float, help="relative step, rad")
    group.add_argument("--to", type=float, help="absolute target, rad")
    parser.add_argument("--duration", type=float, default=1.0,
                        help="slew time for the step, s (default 1.0)")
    args = parser.parse_args()

    rclpy.init()
    client = JointStepClient()
    client.wait_for_state()
    code = client.send(args.joint, args.delta if args.delta is not None
                       else args.to, args.delta is not None, args.duration)
    client.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if code == 0 else 1)


if __name__ == "__main__":
    main()
