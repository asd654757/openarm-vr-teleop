#!/usr/bin/env python3
"""Start only the left OpenArm on one CAN interface."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    description_share = get_package_share_directory("openarm_description")
    bringup_share = get_package_share_directory("openarm_bringup")
    xacro_path = os.path.join(
        description_share,
        "assets", "robot", "openarm_v1.0", "urdf", "openarm_v10.urdf.xacro",
    )
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": "v10",
            "bimanual": "false",
            "arm_prefix": "left",
            "ros2_control": "true",
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
            "can_interface": LaunchConfiguration("can_interface").perform(context),
            "can_fd": "true",
        },
    ).toprettyxml(indent="  ")
    params = {"robot_description": robot_description}
    controllers = os.path.join(
        bringup_share, "config", "controllers", "openarm_left_controllers.yaml"
    )
    enable_effort = (
        LaunchConfiguration("enable_effort_controller").perform(context).lower()
        == "true"
    )
    enable_gravity = (
        LaunchConfiguration("enable_gravity_compensation").perform(context).lower()
        == "true"
    )
    apply_gravity = (
        LaunchConfiguration("apply_gravity_compensation").perform(context).lower()
        == "true"
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[params, controllers],
            output="both",
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "left_forward_position_controller",
                        "left_gripper_controller",
                        "-c", "/controller_manager",
                    ],
                    output="screen",
                )
            ],
        ),
    ]

    if enable_effort or apply_gravity:
        nodes.append(
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "left_forward_effort_controller",
                            "-c", "/controller_manager",
                        ],
                        output="screen",
                    )
                ],
            )
        )

    if enable_gravity or apply_gravity:
        nodes.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="openarm_gravity_comp",
                        executable="gravity_comp_node",
                        name="openarm_gravity_comp",
                        output="screen",
                        parameters=[{
                            "robot_description": robot_description,
                            "enabled": True,
                            "apply_effort": apply_gravity,
                            "enable_left": True,
                            "enable_right": False,
                            "left_base_link": "openarm_left_link0",
                            "gravity_scale": float(
                                LaunchConfiguration("gravity_scale").perform(context)
                            ),
                            "ramp_duration_s": float(
                                LaunchConfiguration(
                                    "gravity_ramp_duration_s"
                                ).perform(context)
                            ),
                            "state_timeout_s": 0.1,
                            "publish_rate_hz": 100.0,
                            "torque_limits": [
                                2.0, 2.0, 2.0, 2.0, 0.5, 0.5, 0.5
                            ],
                        }],
                    )
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("can_interface", default_value="can1"),
        DeclareLaunchArgument("enable_effort_controller", default_value="false"),
        DeclareLaunchArgument(
            "enable_gravity_compensation", default_value="false"
        ),
        DeclareLaunchArgument(
            "apply_gravity_compensation", default_value="false"
        ),
        DeclareLaunchArgument("gravity_scale", default_value="0.01"),
        DeclareLaunchArgument("gravity_ramp_duration_s", default_value="10.0"),
        OpaqueFunction(function=_nodes),
    ])
