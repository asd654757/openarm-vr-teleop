#!/usr/bin/env python3
"""Launch file for openarmx VR teleoperation simulation with fake hardware.

Uses openarmx_description URDF + openarmx_arm_driver (Python) IK solver.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_robot_and_control(context: LaunchContext, controllers_file):
    """Process openarmx xacro and spawn robot_state_publisher + ros2_control_node."""

    description_pkg = get_package_share_directory("openarmx_description")
    xacro_path = os.path.join(
        description_pkg, "urdf", "robot", "v10.urdf.xacro",
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": "v10",
            "body_type": "v10",
            "bimanual": "true",
            "ros2_control": "true",
            "use_fake_hardware": "true",
            "fake_sensor_commands": "false",
            "hand": "true",
        },
    ).toprettyxml(indent="  ")

    robot_description_param = {"robot_description": robot_description}
    controllers_file_str = context.perform_substitution(controllers_file)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description_param],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description_param, controllers_file_str],
        ),
    ]


def generate_launch_description():
    teleop_pkg_share = FindPackageShare("openarmx_teleop_vr")
    description_pkg_share = get_package_share_directory("openarmx_description")

    controllers_file = PathJoinSubstitution([
        teleop_pkg_share, "config", "openarmx_sim_controllers.yaml",
    ])

    teleop_config_file = PathJoinSubstitution([
        teleop_pkg_share, "config", "teleop_params.yaml",
    ])

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("openarmx_description"),
            "urdf", "robot", "openarmx_robot.urdf",
        ]),
        description="Path to openarmx URDF for IK solver",
    )

    robot_and_control = OpaqueFunction(
        function=spawn_robot_and_control,
        args=[controllers_file],
    )

    all_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "left_forward_position_controller",
            "right_forward_position_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    delayed_spawner = TimerAction(period=5.0, actions=[all_spawner])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d", os.path.join(description_pkg_share, "rviz", "bimanual.rviz"),
        ],
    )

    bridge_node = Node(
        package="openarmx_teleop_bridge_vr",
        executable="openarmx_teleop_bridge_vr_node",
        name="openarmx_teleop_bridge_vr_node",
        output="screen",
    )

    teleop_node = Node(
        package="openarmx_teleop_vr",
        executable="openarmx_teleop_vr_node",
        name="openarmx_teleop_vr_node",
        output="screen",
        parameters=[
            teleop_config_file,
            {"urdf_path": LaunchConfiguration("urdf_path")},
        ],
    )

    return LaunchDescription([
        urdf_path_arg,
        robot_and_control,
        delayed_spawner,
        rviz_node,
        bridge_node,
        teleop_node,
    ])
