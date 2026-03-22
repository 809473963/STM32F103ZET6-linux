from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('stm32_robot_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'serial_config.yaml')

    return LaunchDescription([
        Node(
            package='stm32_robot_bridge',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='stm32_robot_bridge',
            executable='teleop_keyboard_node',
            name='teleop_keyboard_node',
            output='screen',
            prefix=['xterm -e'] # Opens a new terminal for keyboard input if possible
        )
    ])
