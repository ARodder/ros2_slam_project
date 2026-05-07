#
# Created by Magnus Mortensen.
# Intended for offboard SLAM bringup and later manual refinement.
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_slam = LaunchConfiguration("launch_slam")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_laser_static_tf = LaunchConfiguration("publish_laser_static_tf")
    slam_params_file = LaunchConfiguration("slam_params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    scan_topic = LaunchConfiguration("scan_topic")

    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")

    offboard_slam_share = get_package_share_directory("offboard_slam")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    default_slam_params = os.path.join(
        offboard_slam_share, "config", "jetracer_slam_toolbox.yaml"
    )
    default_rviz_config = os.path.join(
        offboard_slam_share, "config", "offboard_slam.rviz"
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        condition=IfCondition(launch_slam),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    laser_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_static_tf",
        condition=IfCondition(publish_laser_static_tf),
        arguments=[
            "--x", laser_x,
            "--y", laser_y,
            "--z", laser_z,
            "--roll", laser_roll,
            "--pitch", laser_pitch,
            "--yaw", laser_yaw,
            "--frame-id", base_frame,
            "--child-frame-id", laser_frame,
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_slam",
            default_value="true",
            description="Start slam_toolbox on the offboard computer.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Start RViz for remote SLAM visualization.",
        ),
        DeclareLaunchArgument(
            "publish_laser_static_tf",
            default_value="false",
            description=(
                "Publish base->laser static TF from the offboard computer if the "
                "JetRacer does not already provide it."
            ),
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params,
            description="SLAM Toolbox parameter file for the JetRacer platform.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config file to use for the offboard session.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time. This should stay false on hardware.",
        ),
        DeclareLaunchArgument(
            "odom_frame",
            default_value="odom",
            description="Expected odometry frame from the JetRacer.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Expected robot base frame from the JetRacer.",
        ),
        DeclareLaunchArgument(
            "laser_frame",
            default_value="laser",
            description="Expected lidar frame from the JetRacer.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="Expected lidar topic coming from the JetRacer.",
        ),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.0"),
        DeclareLaunchArgument("laser_roll", default_value="0.0"),
        DeclareLaunchArgument("laser_pitch", default_value="0.0"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        LogInfo(
            msg=[
                "Offboard SLAM expects JetRacer topics over Wi-Fi: scan=",
                scan_topic,
                ", odom frame=",
                odom_frame,
                ", base frame=",
                base_frame,
                ", laser frame=",
                laser_frame,
            ]
        ),
        LogInfo(
            msg=(
                "If SLAM starts but no scans are processed, verify both odom->base "
                "and base->laser TF are available from the JetRacer or enable "
                "publish_laser_static_tf."
            )
        ),
        laser_static_tf,
        slam_launch,
        rviz,
    ])
