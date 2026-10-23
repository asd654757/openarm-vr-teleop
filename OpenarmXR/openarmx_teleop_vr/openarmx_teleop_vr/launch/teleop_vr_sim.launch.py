#!/usr/bin/env python3
"""Launch file for VR teleoperation simulation with openarm_description + rci IK solver."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import TimerAction, OpaqueFunction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawn_robot_and_control(context: LaunchContext, controllers_file):
    """Process xacro and spawn robot_state_publisher + ros2_control_node."""

    description_pkg = get_package_share_directory("openarm_description")
    xacro_path = os.path.join(
        description_pkg, "assets", "robot", "openarm_v1.0", "urdf",
        "openarm_v10.urdf.xacro",
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
    description_pkg_share = get_package_share_directory("openarm_description")
    rci_pkg_share = FindPackageShare("rci_openarm_controller")

    controllers_file = PathJoinSubstitution([
        teleop_pkg_share, "config", "sim_controllers.yaml",
    ])

    vr_teleop_config = PathJoinSubstitution([
        rci_pkg_share, "config", "vr_teleop.yaml",
    ])

    robot_and_control = OpaqueFunction(
        function=spawn_robot_and_control,
        args=[controllers_file],
    )

    # Spawn all controllers in one spawner call. Types are defined in sim_controllers.yaml.
    # The spawner waits for controller_manager, then loads+configures+activates each.
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

    vr_teleop_node = Node(
        package="rci_openarm_controller",
        executable="vr_teleop_node",
        name="rci_vr_teleop",
        output="screen",
        parameters=[vr_teleop_config],
    )

    return LaunchDescription([
        robot_and_control,
        delayed_spawner,
        rviz_node,
        bridge_node,
        vr_teleop_node,
    ])
