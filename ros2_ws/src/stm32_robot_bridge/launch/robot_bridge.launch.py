from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    udp_port = LaunchConfiguration('udp_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'udp_port',
            default_value='15050',
            description='UDP port receiving Raspberry Pi chassis state packets'
        ),
        Node(
            package='stm32_robot_bridge',
            executable='chassis_state_udp_bridge_node',
            name='chassis_state_udp_bridge_node',
            output='screen',
            parameters=[{
                'udp_port': udp_port,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
            }]
        ),
    ])
