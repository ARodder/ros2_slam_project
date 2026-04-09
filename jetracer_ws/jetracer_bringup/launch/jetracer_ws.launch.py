from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    publish_odom_transform = LaunchConfiguration("publish_odom_transform")
    launch_camera = LaunchConfiguration("launch_camera")
    launch_lidar = LaunchConfiguration("launch_lidar")
    lidar_channel_type = LaunchConfiguration("lidar_channel_type")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_serial_baudrate = LaunchConfiguration("lidar_serial_baudrate")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port_name",
            default_value="/dev/ttyACM0",
            description="Serial device for the JetRacer controller",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Serial baud rate for the JetRacer controller",
        ),
        DeclareLaunchArgument(
            "publish_odom_transform",
            default_value="true",
            description="Whether to publish the odom to base_link transform",
        ),
        DeclareLaunchArgument(
            "launch_camera",
            default_value="false",
            description="Whether to start the camera node",
        ),
        DeclareLaunchArgument(
            "launch_lidar",
            default_value="true",
            description="Whether to start the lidar node",
        ),
        DeclareLaunchArgument(
            "lidar_channel_type",
            default_value="serial",
            description="Lidar channel type",
        ),
        DeclareLaunchArgument(
            "lidar_serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial device for the lidar",
        ),
        DeclareLaunchArgument(
            "lidar_serial_baudrate",
            default_value="115200",
            description="Serial baud rate for the lidar",
        ),
        DeclareLaunchArgument(
            "lidar_frame_id",
            default_value="laser",
            description="Frame id for published lidar scans",
        ),
        DeclareLaunchArgument(
            "lidar_inverted",
            default_value="false",
            description="Whether to invert lidar scan data",
        ),
        DeclareLaunchArgument(
            "lidar_angle_compensate",
            default_value="true",
            description="Whether to enable lidar angle compensation",
        ),
        DeclareLaunchArgument(
            "lidar_scan_mode",
            default_value="Sensitivity",
            description="Scan mode passed to the lidar node",
        ),
        Node(
            package="car_serial_interp",
            executable="jetracer_node",
            name="jetracer",
            output="screen",
            parameters=[{
                "port_name": port_name,
                "baud_rate": baud_rate,
                "publish_odom_transform": publish_odom_transform,
            }],
        ),
        Node(
            package="camera_node",
            executable="camera_node",
            name="camera_node",
            output="screen",
            condition=IfCondition(launch_camera),
        ),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            condition=IfCondition(launch_lidar),
            parameters=[{
                "channel_type": lidar_channel_type,
                "serial_port": lidar_serial_port,
                "serial_baudrate": lidar_serial_baudrate,
                "frame_id": lidar_frame_id,
                "inverted": lidar_inverted,
                "angle_compensate": lidar_angle_compensate,
                "scan_mode": lidar_scan_mode,
            }],
        ),
    ])
