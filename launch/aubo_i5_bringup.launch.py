# NOTE: Rename this file to aubo_i5_bringup.launch.py for ros2 launch to pick it up.
#
# Usage:
#   ros2 launch aubo_driver aubo_i5_bringup.launch.py robot_ip:=192.168.1.102
#
# This launch file starts:
#   1. robot_state_publisher  — publishes TF from the URDF
#   2. controller_manager     — loads the aubo_driver hardware plugin
#   3. joint_state_broadcaster — bridges hardware state to /joint_states

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.102",
        description="IP address of the Aubo robot controller",
    )

    robot_ip = LaunchConfiguration("robot_ip")

    # -------------------------------------------------------------------------
    # Build robot_description from the URDF/xacro.
    # Adjust the package name and xacro file path to match your setup.
    # -------------------------------------------------------------------------
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_description"), "urdf", "aubo_i5.urdf.xacro"]
            ),
            " robot_ip:=", robot_ip,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # -------------------------------------------------------------------------
    # robot_state_publisher — publishes TF from robot_description
    # -------------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # -------------------------------------------------------------------------
    # controller_manager — loads the aubo_driver hardware plugin and controllers
    # from the ros2_control URDF tag.
    # -------------------------------------------------------------------------
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare("aubo_driver"), "config", "controllers.yaml"]
            ),
        ],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # joint_state_broadcaster — publishes /joint_states from hardware state
    # -------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # -------------------------------------------------------------------------
    # joint_trajectory_controller — receives MoveIt trajectory goals
    # -------------------------------------------------------------------------
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawn the trajectory controller only after the broadcaster is up
    delay_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            delay_trajectory_controller,
        ]
    )
