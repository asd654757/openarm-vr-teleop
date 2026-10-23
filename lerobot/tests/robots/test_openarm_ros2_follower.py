from types import SimpleNamespace

import pytest

from lerobot.robots import make_robot_from_config
from lerobot.robots.openarm_ros2_follower import (
    BiOpenArmROS2Follower,
    BiOpenArmROS2FollowerConfig,
    OpenArmROS2Follower,
    OpenArmROS2FollowerConfig,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _ActionClient:
    def __init__(self):
        self.goals = []

    def wait_for_server(self, timeout_sec):
        del timeout_sec
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)


class _GripperAction:
    class Goal:
        def __init__(self):
            self.command = SimpleNamespace(position=0.0, max_effort=0.0)


def _connected_robot() -> OpenArmROS2Follower:
    robot = OpenArmROS2Follower(OpenArmROS2FollowerConfig(include_gripper=True))
    robot._connected = True
    robot._arm_command_pub = _Publisher()
    robot._gripper_action_client = _ActionClient()
    robot._gripper_action_type = _GripperAction
    robot._rclpy = SimpleNamespace(spin_once=lambda *_args, **_kwargs: None)
    return robot


def test_factory_creates_openarm_ros2_followers():
    assert isinstance(make_robot_from_config(OpenArmROS2FollowerConfig()), OpenArmROS2Follower)
    assert isinstance(make_robot_from_config(BiOpenArmROS2FollowerConfig()), BiOpenArmROS2Follower)


def test_openarm_ros2_features_match_existing_openarm_names():
    robot = OpenArmROS2Follower(OpenArmROS2FollowerConfig())

    assert list(robot.action_features) == [
        "joint_1.pos",
        "joint_2.pos",
        "joint_3.pos",
        "joint_4.pos",
        "joint_5.pos",
        "joint_6.pos",
        "joint_7.pos",
        "gripper.pos",
    ]
    assert robot.observation_features == robot.action_features


def test_send_action_converts_degrees_to_ros_units():
    robot = _connected_robot()

    sent = robot.send_action(
        {
            "joint_1.pos": 180.0,
            "joint_2.pos": 30.0,
            "gripper.pos": -65.0,
        }
    )

    assert sent["joint_1.pos"] == 75.0
    assert sent["joint_2.pos"] == 30.0
    assert sent["gripper.pos"] == -65.0

    arm_msg = robot._arm_command_pub.messages[-1]
    assert arm_msg.data[0] == pytest.approx(75.0 * 3.141592653589793 / 180.0)
    assert arm_msg.data[1] == pytest.approx(30.0 * 3.141592653589793 / 180.0)
    assert robot._gripper_action_client.goals[-1].command.position == pytest.approx(0.044)


def test_get_observation_converts_ros_units_to_degrees():
    robot = _connected_robot()
    robot._latest_positions = {
        "openarm_joint1": 3.141592653589793 / 2,
        "openarm_joint2": 0.0,
        "openarm_joint3": 0.0,
        "openarm_joint4": 0.0,
        "openarm_joint5": 0.0,
        "openarm_joint6": 0.0,
        "openarm_joint7": 0.0,
        "openarm_finger_joint1": 0.022,
    }

    obs = robot.get_observation()

    assert obs["joint_1.pos"] == pytest.approx(90.0)
    assert obs["gripper.pos"] == pytest.approx(-32.5)


def test_bimanual_defaults_match_openarm_ros2_controller_names():
    robot = BiOpenArmROS2Follower(BiOpenArmROS2FollowerConfig())

    assert list(robot.action_features)[:2] == ["right_joint_1.pos", "right_joint_2.pos"]
    assert "left_joint_1.pos" in robot.action_features
    assert robot.left_arm.config.command_topic == "left_forward_position_controller/commands"
    assert robot.right_arm.config.command_topic == "right_forward_position_controller/commands"
