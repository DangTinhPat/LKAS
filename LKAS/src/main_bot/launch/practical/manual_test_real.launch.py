import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Manual drivetrain bring-up for real-hardware bench testing — same controller/MCU stack as
# robot.launch.py, but WITHOUT the autonomous pipeline (lane_follower_node, overtake_vision_node,
# lane_control_node, overtake_node), so /cmd_vel is free for a teleop source (e.g. gui's
# control_gui joystick) instead of being fought over by lane_control_node. Use this to verify
# motor/servo/encoder/IMU wiring and calibration before ever running the full autonomous stack.
# Sim counterpart: simulation/manual_test_sim.launch.py.


def generate_launch_description():

    pkg_path = get_package_share_directory('main_bot')

    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_file, mappings={'sim_mode': 'false'}
    ).toxml()

    controller_config = os.path.join(pkg_path, 'config', 'practical', 'controller_real.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen',
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_config],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen',
    )

    # Twist (/cmd_vel) -> TwistStamped (/ackermann_steering_controller/reference)
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('cmd_vel_in',  '/cmd_vel'),
            ('cmd_vel_out', '/ackermann_steering_controller/reference'),
        ],
        output='screen',
    )

    # Owns the serial connection to the ESP32; bridges it into /mcu/joint_states,
    # /mcu/joint_commands and /imu. See mcu_agent/README.md for the topic contract and the
    # one-time setup script required before this will actually connect to hardware.
    mcu_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('mcu_agent'), 'launch', 'mcu_agent.launch.py')
        ])
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        mcu_agent_launch,
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=5.0, actions=[ackermann_steering_controller_spawner]),
        TimerAction(period=6.0, actions=[twist_stamper_node]),
    ])
