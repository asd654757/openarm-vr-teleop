#!/usr/bin/env python

"""Smoke-test OpenArm ROS2 fake hardware through the LeRobot robot API.

Before running this script, start OpenArm ROS2 with the forward position
controller, for example:

Single arm:
    ros2 launch openarm_bringup openarm.launch.py \
        use_fake_hardware:=true robot_controller:=forward_position_controller

Bimanual:
    ros2 launch openarm_bringup openarm.bimanual.launch.py \
        use_fake_hardware:=true robot_controller:=forward_position_controller
"""

import argparse
import math
import time

from lerobot.robots.openarm_ros2_follower import (
    BiOpenArmROS2Follower,
    BiOpenArmROS2FollowerConfig,
    OpenArmROS2Follower,
    OpenArmROS2FollowerConfig,
)
from lerobot.utils.robot_utils import precise_sleep


def build_action(feature_names: list[str], elapsed_s: float, amplitude_deg: float) -> dict[str, float]:
    action = dict.fromkeys(feature_names, 0.0)
    wave = amplitude_deg * math.sin(elapsed_s)

    for key in action:
        if key.endswith("joint_2.pos"):
            action[key] = wave
        elif key.endswith("joint_4.pos"):
            action[key] = 30.0 + 0.5 * wave
        elif key.endswith("gripper.pos"):
            action[key] = -20.0

    return action


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bimanual", action="store_true")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--amplitude-deg", type=float, default=15.0)
    args = parser.parse_args()

    if args.bimanual:
        config = BiOpenArmROS2FollowerConfig(namespace=args.namespace)
        config.left_arm_config.include_gripper = not args.no_gripper
        config.right_arm_config.include_gripper = not args.no_gripper
        robot = BiOpenArmROS2Follower(config)
    else:
        robot = OpenArmROS2Follower(
            OpenArmROS2FollowerConfig(namespace=args.namespace, include_gripper=not args.no_gripper)
        )

    robot.connect(calibrate=False)
    try:
        start = time.perf_counter()
        next_print_s = 0.0
        while True:
            elapsed_s = time.perf_counter() - start
            if elapsed_s >= args.duration_s:
                break

            action = build_action(list(robot.action_features), elapsed_s, args.amplitude_deg)
            sent = robot.send_action(action)
            obs = robot.get_observation()

            if elapsed_s >= next_print_s:
                sample_key = next(iter(sent))
                print(
                    f"t={elapsed_s:5.2f}s sent {sample_key}={sent[sample_key]: .2f}, "
                    f"obs keys={len(obs)}"
                )
                next_print_s += 1.0

            precise_sleep(1.0 / args.fps)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
