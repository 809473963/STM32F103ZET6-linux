from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("stm32_serial_bridge"),
        "config",
        "bridge_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="stm32_serial_bridge",
            executable="serial_bridge_node",
            name="stm32_serial_bridge",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="stm32_serial_bridge",
            executable="tuning_slider_gui",
            name="tuning_slider_gui",
            output="screen",
        ),
    ])