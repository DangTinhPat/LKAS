import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Real-robot bringup — the practical counterpart of simulation/gazebo.launch.py. Same node
# graph (lane_follower_node, overtake_vision_node, lane_control_node, overtake_node) driven by
# the same ackermann_steering_controller, but robot_description is built with sim_mode:=false
# (main_bot_hardware/RealRobotSystem instead of gz_ros2_control) and there is no Gazebo — sensor
# and MCU I/O come from real drivers / mcu_agent instead.


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

    # No Gazebo here, so controller_manager must be launched explicitly (in sim this is done
    # internally by the gz_ros2_control plugin loaded from simulation/ros2_control.xacro).
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

    ekf_config = os.path.join(pkg_path, 'config', 'practical', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odometry/filtered')],
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

    lane_follower_node = Node(
        package='main_bot',
        executable='lane_follower_node.py',
        name='lane_follower_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    overtake_vision_node = Node(
        package='main_bot',
        executable='overtake_vision_node.py',
        name='overtake_vision_node',
        parameters=[{
            'use_sim_time':  False,
            'publish_debug': True,
        }],
        output='screen',
    )

    lane_control_node = Node(
        package='main_bot',
        executable='lane_control_node',
        name='lane_control_node',
        parameters=[{
            'use_sim_time': False,
            'speed':      1.0,
            'k':          1.0,
            'max_steer':  0.52,
            'timeout':    0.5,
            'kappa_blend': 0.85,
        }],
        output='screen',
    )

    # Overtake pipeline (Layers 2-7). See README for the tunable-parameter reference.
    overtake_node = Node(
        package='main_bot',
        executable='overtake_node',
        name='overtake_node',
        parameters=[{
            'use_sim_time':        False,
            'front_detect_range':  1.0,
            'front_safe_min':      0.35,
            'front_sector_deg':   21.5,
            'adjacent_clear_min':  0.30,
            'npc_speed':           0.25,
            'gap_time_threshold':  8.0,
            'prepare_hold_time':   0.5,
            'overtake_hold_time':  6.0,
            'return_tol':          0.04,
            'return_hold_time':    3.0,
            'overtake_offset':    -0.534,
            'offset_rate_limit':   0.45,
            'return_rate_limit':   0.30,
            'abort_front_dist':    0.10,
            'imu_ay_limit':        5.0,
            'same_lane_half_width': 0.15,
            'normal_speed':        0.9,
            'follow_speed':        0.25,
            'creep_speed':         0.15,
        }],
        output='screen',
    )

    # ── Real sensor drivers ─────────────────────────────────────────────────────
    # TODO: hardware/driver not chosen yet — add the matching Node(...) once it is.
    # Camera must publish sensor_msgs/Image on /camera/image_raw, e.g. v4l2_camera:
    #   Node(
    #       package='v4l2_camera', executable='v4l2_camera_node', name='camera_driver',
    #       parameters=[{'image_size': [640, 480]}],
    #       remappings=[('/image_raw', '/camera/image_raw')],
    #       output='screen',
    #   )
    # LiDAR must publish sensor_msgs/LaserScan on /scan, e.g. rplidar_ros:
    #   Node(
    #       package='rplidar_ros', executable='rplidar_node', name='lidar_driver',
    #       parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'lidar_frame'}],
    #       output='screen',
    #   )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        mcu_agent_launch,
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=5.0, actions=[ackermann_steering_controller_spawner]),
        TimerAction(period=6.0, actions=[twist_stamper_node, ekf_node]),
        TimerAction(period=7.0, actions=[lane_follower_node, lane_control_node,
                                          overtake_node, overtake_vision_node]),
    ])
