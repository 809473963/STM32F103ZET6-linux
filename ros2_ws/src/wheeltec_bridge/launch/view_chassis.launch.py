#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("wheeltec_bridge")
    urdf_file = os.path.join(pkg_share, "urdf", "wheeltec_chassis.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "wheeltec_chassis.rviz")

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package="wheeltec_bridge",
            executable="wheeltec_chassis_marker",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
