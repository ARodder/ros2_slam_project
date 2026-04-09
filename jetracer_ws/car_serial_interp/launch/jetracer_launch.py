from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('car_serial_interp')
    params = os.path.join(pkg_share, 'config', 'custom_ekf.yaml')

    return LaunchDescription([
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
                '--x', '0.12', '--y', '0', '--z', '0.10',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser'
            ]
        )
    ])