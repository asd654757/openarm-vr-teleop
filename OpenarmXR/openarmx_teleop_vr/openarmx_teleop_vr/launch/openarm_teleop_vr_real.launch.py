#!/usr/bin/env python3
"""Launch file for real openarm robot VR teleoperation.

Assumes openarm_bringup is already running in a separate terminal:
  ros2 launch openarm_bringup openarm.bimanual.launch.py \
      use_fake_hardware:=false \
      robot_controller:=forward_position_controller \
      right_can_interface:=can0 \
      left_can_interface:=can1

This launch starts:
  1. joint_name_remapper: openarm_ → openarmx_ joint name translation
  2. openarmx_teleop_bridge_vr_node: VR headset UDP → ROS2 topics
  3. openarmx_teleop_vr_node: IK computation → joint commands

Topic flow:
  /joint_states (from bringup, openarm_ prefix)
      → remapper reads, converts names
      → /joint_states_remapped (openarmx_ prefix)
      → teleop node reads (via remap /joint_states := /joint_states_remapped)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teleop_pkg_share = FindPackageShare("openarmx_teleop_vr")

    teleop_config_file = PathJoinSubstitution([
        teleop_pkg_share, "config", "teleop_params.yaml",
    ])

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("openarm_description"),
            "assets", "robot", "openarm_v1.0", "urdf",
            "openarm_v10_bimanual_openarmx_compat.urdf",
        ]),
        description="openarmx-compatible openarm URDF for IK solver",
    )

    # Reads /joint_states (openarm_ prefix from bringup)
    # Publishes /joint_states_remapped (openarmx_ prefix)
    joint_name_remapper = Node(
        package="openarmx_teleop_vr",
        executable="joint_name_remapper",
        name="joint_name_remapper",
        output="screen",
        parameters=[{
            "input_topic": "/joint_states",
            "output_topic": "/joint_states_remapped",
            "from_prefix": "openarm_",
            "to_prefix": "openarmx_",
        }],
    )

    bridge_node = Node(
        package="openarmx_teleop_bridge_vr",
        executable="openarmx_teleop_bridge_vr_node",
        name="openarmx_teleop_bridge_vr_node",
        output="screen",
    )

    # Teleop node subscribes to /joint_states_remapped instead of /joint_states
    teleop_node = Node(
        package="openarmx_teleop_vr",
        executable="openarmx_teleop_vr_node",
        name="openarmx_teleop_vr_node",
        output="screen",
        parameters=[
            teleop_config_file,
            {
                "urdf_path": LaunchConfiguration("urdf_path"),
                "separate_gripper_controllers": True,
            },
        ],
        remappings=[
            ("/joint_states", "/joint_states_remapped"),
        ],
    )

    return LaunchDescription([
        urdf_path_arg,
        joint_name_remapper,
        bridge_node,
        teleop_node,
    ])
