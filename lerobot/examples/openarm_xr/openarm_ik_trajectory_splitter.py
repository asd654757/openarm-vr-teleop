#!/usr/bin/env python

"""Split teleop_xr OpenArm IK JointTrajectory output into left/right controllers."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LEFT_JOINTS = [
    "openarm_left_joint1",
    "openarm_left_joint2",
    "openarm_left_joint3",
    "openarm_left_joint4",
    "openarm_left_joint5",
    "openarm_left_joint6",
    "openarm_left_joint7",
]

RIGHT_JOINTS = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]


def _resolve_topic(topic: str) -> str:
    return topic if topic.startswith("/") else f"/{topic}"


class OpenArmIKTrajectorySplitter(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("openarm_ik_trajectory_splitter")
        self.left_pub = self.create_publisher(
            JointTrajectory, _resolve_topic(args.left_output_topic), 10
        )
        self.right_pub = self.create_publisher(
            JointTrajectory, _resolve_topic(args.right_output_topic), 10
        )
        self.create_subscription(
            JointTrajectory,
            _resolve_topic(args.input_topic),
            self._on_trajectory,
            10,
        )
        self.get_logger().info(
            f"Splitting {args.input_topic} to {args.left_output_topic} and {args.right_output_topic}"
        )

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.joint_names)}
        self._publish_subset(msg, LEFT_JOINTS, name_to_index, self.left_pub)
        self._publish_subset(msg, RIGHT_JOINTS, name_to_index, self.right_pub)

    def _publish_subset(self, msg, joint_names, name_to_index, publisher) -> None:
        if not all(name in name_to_index for name in joint_names):
            missing = [name for name in joint_names if name not in name_to_index]
            self.get_logger().warning(f"Skipping trajectory; missing joints: {missing}")
            return

        out = JointTrajectory()
        out.header = msg.header
        out.joint_names = list(joint_names)

        for point in msg.points:
            out_point = JointTrajectoryPoint()
            indices = [name_to_index[name] for name in joint_names]
            if point.positions:
                out_point.positions = [point.positions[idx] for idx in indices]
            if point.velocities:
                out_point.velocities = [point.velocities[idx] for idx in indices]
            if point.accelerations:
                out_point.accelerations = [point.accelerations[idx] for idx in indices]
            if point.effort:
                out_point.effort = [point.effort[idx] for idx in indices]
            out_point.time_from_start = point.time_from_start
            out.points.append(out_point)

        publisher.publish(out)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/joint_trajectory")
    parser.add_argument(
        "--left-output-topic",
        default="/left_joint_trajectory_controller/joint_trajectory",
    )
    parser.add_argument(
        "--right-output-topic",
        default="/right_joint_trajectory_controller/joint_trajectory",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = OpenArmIKTrajectorySplitter(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
