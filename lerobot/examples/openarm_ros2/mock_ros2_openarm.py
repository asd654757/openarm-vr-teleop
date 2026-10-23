#!/usr/bin/env python

"""Minimal ROS2 OpenArm mock for testing the LeRobot ROS2 bridge.

This node intentionally avoids ros2_control. It exposes the same topics/actions
that `OpenArmROS2Follower` expects, so the LeRobot bridge can be tested before
the full OpenArm ROS2 workspace and controller dependencies are installed.
"""

import argparse
from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


SINGLE_JOINTS = [f"openarm_joint{i}" for i in range(1, 8)]
LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]


class MockOpenArmROS2(Node):
    def __init__(self, bimanual: bool, fps: float, include_gripper: bool):
        super().__init__("mock_openarm_ros2")
        self._lock = Lock()
        self._bimanual = bimanual
        self._include_gripper = include_gripper

        if include_gripper:
            from control_msgs.action import GripperCommand
            from rclpy.action import ActionServer
            self._gripper_action_type = GripperCommand
            self._action_server_type = ActionServer

        if bimanual:
            self._joint_names = list(RIGHT_JOINTS)
            if include_gripper:
                self._joint_names += ["openarm_right_finger_joint1"]
            self._joint_names += list(LEFT_JOINTS)
            if include_gripper:
                self._joint_names += ["openarm_left_finger_joint1"]
            self._positions = dict.fromkeys(self._joint_names, 0.0)
            self.create_subscription(
                Float64MultiArray,
                "right_forward_position_controller/commands",
                lambda msg: self._set_arm(RIGHT_JOINTS, msg),
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                "left_forward_position_controller/commands",
                lambda msg: self._set_arm(LEFT_JOINTS, msg),
                10,
            )
            if include_gripper:
                self._right_gripper_server = self._action_server_type(
                    self,
                    self._gripper_action_type,
                    "right_gripper_controller/gripper_cmd",
                    self._set_right_gripper,
                )
                self._left_gripper_server = self._action_server_type(
                    self,
                    self._gripper_action_type,
                    "left_gripper_controller/gripper_cmd",
                    self._set_left_gripper,
                )
        else:
            self._joint_names = list(SINGLE_JOINTS)
            if include_gripper:
                self._joint_names += ["openarm_finger_joint1"]
            self._positions = dict.fromkeys(self._joint_names, 0.0)
            self.create_subscription(
                Float64MultiArray,
                "forward_position_controller/commands",
                lambda msg: self._set_arm(SINGLE_JOINTS, msg),
                10,
            )
            if include_gripper:
                self._gripper_server = self._action_server_type(
                    self,
                    self._gripper_action_type,
                    "gripper_controller/gripper_cmd",
                    self._set_single_gripper,
                )

        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / fps, self._publish_joint_states)

    def _set_arm(self, joint_names: list[str], msg: Float64MultiArray) -> None:
        with self._lock:
            for name, position in zip(joint_names, msg.data, strict=False):
                self._positions[name] = float(position)

    def _set_gripper(self, joint_name: str, goal_handle):
        with self._lock:
            self._positions[joint_name] = float(goal_handle.request.command.position)
        goal_handle.succeed()
        result = self._gripper_action_type.Result()
        result.position = self._positions[joint_name]
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result

    def _set_single_gripper(self, goal_handle):
        return self._set_gripper("openarm_finger_joint1", goal_handle)

    def _set_right_gripper(self, goal_handle):
        return self._set_gripper("openarm_right_finger_joint1", goal_handle)

    def _set_left_gripper(self, goal_handle):
        return self._set_gripper("openarm_left_finger_joint1", goal_handle)

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.name = list(self._joint_names)
            msg.position = [self._positions[name] for name in self._joint_names]
            msg.velocity = [0.0] * len(msg.name)
            msg.effort = [0.0] * len(msg.name)
        self._publisher.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bimanual", action="store_true")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--fps", type=float, default=100.0)
    args = parser.parse_args()

    rclpy.init()
    node = MockOpenArmROS2(bimanual=args.bimanual, fps=args.fps, include_gripper=not args.no_gripper)
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
