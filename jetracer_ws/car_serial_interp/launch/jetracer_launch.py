#
# Created by Aleksander Røder on 09/04/2026.
#

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('car_serial_interp')
    params = os.path.join(pkg_share, 'config', 'custom_ekf.yaml')
    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_log_output_directory',
            default_value='odom_logs',
            description='Directory for odometry CSV logs written when the logger node stops.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'serial_port': '/dev/ttyACM1',
            }.items()
        ),
        Node(
            package='car_serial_interp',
            executable='jetracer_node',
            name='jetracer_node',
            output='screen',
            parameters=[{
                'publish_odom_transform': False
            }]
        ),
        Node(
            package='car_serial_interp',
            executable='custom_ekf_node',
            name='custom_ekf_node',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='car_serial_interp',
            executable='odom_csv_logger_node',
            name='odom_csv_logger_node',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'filtered_topic': '/odometry/filtered',
                'output_directory': LaunchConfiguration('odom_log_output_directory')
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.08',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_imu_link'
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--x', '0.0', '--y', '0', '--z', '0.10',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ]
        )
    ])
