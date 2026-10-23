#!/usr/bin/env python

"""Publish a tiny fake OpenArm JointTrajectory for Pico/MuJoCo smoke tests."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lerobot.robots.openarm_ros2_follower.config_openarm_ros2_follower import (
    OPENARM_LEFT_JOINT_NAMES,
    OPENARM_RIGHT_JOINT_NAMES,
)


class FakeOpenArmTrajectoryPublisher(Node):
    def __init__(self, topic: str, fps: float):
        super().__init__("fake_openarm_trajectory_publisher")
        self._pub = self.create_publisher(JointTrajectory, topic, 10)
        self._start = time.perf_counter()
        self._timer = self.create_timer(1.0 / fps, self._publish)

    def _publish(self) -> None:
        t = time.perf_counter() - self._start
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(OPENARM_RIGHT_JOINT_NAMES) + list(OPENARM_LEFT_JOINT_NAMES)

        right = [
            0.35 * math.sin(t),
            0.25 + 0.20 * math.sin(t * 0.7),
            0.15 * math.sin(t * 0.9),
            1.40 + 0.12 * math.sin(t * 0.5),
            0.12 * math.sin(t * 1.2),
            0.08 * math.cos(t),
            0.10 * math.sin(t * 0.6),
        ]
        left = [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        point = JointTrajectoryPoint()
        point.positions = right + left
        msg.points = [point]
        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/joint_trajectory")
    parser.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = FakeOpenArmTrajectoryPublisher(args.topic, args.fps)
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
