import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    map_topic = LaunchConfiguration("map_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    absolute_goal_topic = LaunchConfiguration("absolute_goal_topic")
    relative_goal_topic = LaunchConfiguration("relative_goal_topic")
    path_topic = LaunchConfiguration("path_topic")
    navigation_mode = LaunchConfiguration("navigation_mode")

    package_share = get_package_share_directory("offboard_navigation")
    default_rviz_config = os.path.join(package_share, "config", "offboard_navigation.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_navigation",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Start RViz for navigation bringup.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config for offboard navigation bringup.",
        ),
        DeclareLaunchArgument(
            "navigation_mode",
            default_value="mapping",
            description=(
                "High-level operating mode. Intended values are mapping for live "
                "SLAM-assisted driving or localization for navigation against a "
                "saved map."
            ),
        ),
        DeclareLaunchArgument(
            "map_topic",
            default_value="/map",
            description="Map input that the future planner should consume.",
        ),
        DeclareLaunchArgument(
            "pose_topic",
            default_value="/pose",
            description="Robot pose estimate from SLAM or localization.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Velocity command topic sent back to the JetRacer.",
        ),
        DeclareLaunchArgument(
            "absolute_goal_topic",
            default_value="/goal_pose",
            description="Future absolute goal interface in map coordinates.",
        ),
        DeclareLaunchArgument(
            "relative_goal_topic",
            default_value="/goal_relative",
            description=(
                "Future relative motion interface, e.g. move X meters and rotate "
                "by Y radians from the current pose."
            ),
        ),
        DeclareLaunchArgument(
            "path_topic",
            default_value="/planned_path",
            description="Future planned path output topic.",
        ),
        LogInfo(
            msg=(
                "Offboard navigation structure only: no planner/controller node is "
                "started yet."
            )
        ),
        LogInfo(
            msg=[
                "Expected inputs: map=",
                map_topic,
                ", pose=",
                pose_topic,
                ", absolute goal=",
                absolute_goal_topic,
                ", relative goal=",
                relative_goal_topic,
            ]
        ),
        LogInfo(
            msg=[
                "Expected outputs: path=",
                path_topic,
                ", cmd_vel back to JetRacer=",
                cmd_vel_topic,
                ", mode=",
                navigation_mode,
            ]
        ),
        LogInfo(
            msg=(
                "Recommended next implementation is a goal-handler/controller "
                "package that converts absolute or relative goals into a path and "
                "velocity commands using the live SLAM pose and map."
            )
        ),
        rviz,
    ])
