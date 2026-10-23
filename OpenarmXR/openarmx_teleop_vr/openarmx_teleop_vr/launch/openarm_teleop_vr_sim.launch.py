#!/usr/bin/env python3
"""Launch file for openarm model VR teleoperation simulation.

Single URDF approach: openarm kinematics/dynamics/mesh with openarmx_ naming.
- ros2_control, RViz, IK solver all use the same converted URDF.
- Existing openarmx teleop Python node runs without any code changes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teleop_pkg_share = FindPackageShare("openarmx_teleop_vr")
    openarm_desc_share = get_package_share_directory("openarm_description")

    # Single converted URDF: openarm body + openarmx_ naming + ros2_control fake hw
    sim_urdf_path = os.path.join(
        openarm_desc_share, "assets", "robot", "openarm_v1.0", "urdf",
        "openarm_v10_bimanual_openarmx_compat_sim.urdf",
    )
    with open(sim_urdf_path, 'r') as f:
        robot_description = f.read()

    robot_description_param = {"robot_description": robot_description}

    # IK solver URDF (same openarm kinematics, without ros2_control tags)
    ik_urdf_path = os.path.join(
        openarm_desc_share, "assets", "robot", "openarm_v1.0", "urdf",
        "openarm_v10_bimanual_openarmx_compat.urdf",
    )

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=ik_urdf_path,
        description="URDF for IK solver",
    )

    controllers_file = PathJoinSubstitution([
        teleop_pkg_share, "config", "openarmx_sim_controllers.yaml",
    ])

    teleop_config_file = PathJoinSubstitution([
        teleop_pkg_share, "config", "teleop_params.yaml",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file],
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
            "-d", os.path.join(openarm_desc_share, "rviz", "bimanual.rviz"),
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
        robot_state_publisher,
        ros2_control_node,
        delayed_spawner,
        rviz_node,
        bridge_node,
        teleop_node,
    ])
