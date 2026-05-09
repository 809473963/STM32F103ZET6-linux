#!/usr/bin/env python3
"""
wheeltec_teleop.launch.py
=========================
在树莓派上启动：
  ros2 launch wheeltec_bridge wheeltec_teleop.launch.py

参数:
  port   := /dev/ttyUSB0    串口设备
  baud   := 115200          波特率
  keyboard := true          是否同时启动键盘节点（需要 TTY，建议 false 后单开终端）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0",
        description="WHEELTEC 串口设备路径")
    baud_arg = DeclareLaunchArgument(
        "baud", default_value="115200",
        description="波特率")

    serial_node = Node(
        package="wheeltec_bridge",
        executable="wheeltec_serial",
        name="wheeltec_serial_node",
        parameters=[{
            "port":       LaunchConfiguration("port"),
            "baud":       LaunchConfiguration("baud"),
            "lin_thresh": 0.05,
            "ang_thresh": 0.10,
            "watchdog_s": 0.5,
        }],
        output="screen",
    )

    hint = LogInfo(msg=(
        "\n"
        "==================================================\n"
        "  串口节点已启动。键盘控制请另开终端运行：\n"
        "  ros2 run wheeltec_bridge wheeltec_keyboard\n"
        "  或从 PC 通过 SSH 执行同样命令\n"
        "==================================================\n"
    ))

    return LaunchDescription([
        port_arg,
        baud_arg,
        serial_node,
        hint,
    ])
