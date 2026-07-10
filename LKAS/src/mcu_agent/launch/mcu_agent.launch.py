from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate',   default_value='115200'),

        Node(
            package='mcu_agent',
            executable='mcu_agent_node',
            name='mcu_agent_node',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate':   LaunchConfiguration('baud_rate'),
            }],
            output='screen',
        ),
    ])
