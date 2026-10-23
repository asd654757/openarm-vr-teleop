#!/usr/bin/env python3
"""OpenarmXR bridge and IK configured for left-arm-only hardware."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    teleop_share = get_package_share_directory("openarmx_teleop_vr")
    description_share = get_package_share_directory("openarm_description")
    return LaunchDescription([
        Node(
            package="openarmx_teleop_vr",
            executable="joint_name_remapper",
            parameters=[{
                "input_topic": "/joint_states",
                "output_topic": "/joint_states_remapped",
                "from_prefix": "openarm_",
                "to_prefix": "openarmx_",
            }],
            output="screen",
        ),
        Node(
            package="openarmx_teleop_bridge_vr",
            executable="openarmx_teleop_bridge_vr_node",
            output="screen",
        ),
        Node(
            package="openarmx_teleop_vr",
            executable="openarmx_teleop_vr_node",
            parameters=[
                os.path.join(teleop_share, "config", "teleop_params.yaml"),
                {
                    "urdf_path": os.path.join(
                        description_share, "assets", "robot", "openarm_v1.0",
                        "urdf", "openarm_v10_bimanual_openarmx_compat.urdf",
                    ),
                    "active_arm": "left",
                    "separate_gripper_controllers": True,
                },
            ],
            remappings=[("/joint_states", "/joint_states_remapped")],
            output="screen",
        ),
    ])
