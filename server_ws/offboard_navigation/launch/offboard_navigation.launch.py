import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    map_name = LaunchConfiguration("map_name")
    posegraph_file = LaunchConfiguration("posegraph_file")
    start_at_dock = LaunchConfiguration("start_at_dock")

    package_share = get_package_share_directory("offboard_navigation")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    offboard_slam_share = get_package_share_directory("offboard_slam")
    repo_maps_dir = os.path.join(offboard_slam_share, "maps")

    default_rviz_config = os.path.join(package_share, "config", "offboard_navigation.rviz")
    default_nav2_params = os.path.join(package_share, "config", "nav2_params.yaml")
    default_slam_params = os.path.join(package_share, "config", "slam_localization.yaml")
    default_posegraph = os.path.join(repo_maps_dir, "current", "current")
    posegraph_from_map_name = PythonExpression(
        [
            "'",
            default_posegraph,
            "' if '",
            map_name,
            "' == '' else '",
            repo_maps_dir,
            "/",
            map_name,
            "/",
            map_name,
            "'",
        ]
    )

    slam_localization = LifecycleNode(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": use_lifecycle_manager,
                "map_file_name": posegraph_from_map_name,
                "map_start_pose": [0.0, 0.0, 0.0],
            },
        ],
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_localization),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] slam_toolbox localization is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_localization),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": nav2_params_file,
            "use_composition": "False",
        }.items(),
    )

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
            default_value="true",
            description="Start RViz for navigation bringup.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config for offboard navigation bringup.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time. This should stay false on hardware.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Autostart slam_toolbox localization and Nav2 lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "use_lifecycle_manager",
            default_value="false",
            description="Enable slam_toolbox bond-managed lifecycle mode.",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params,
            description="Nav2 parameter file for the JetRacer platform.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params,
            description="slam_toolbox localization parameter file.",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="",
            description=(
                "Optional saved map folder name under offboard_slam/maps, for "
                "example classroom or hallway. Leave empty to use maps/current."
            ),
        ),
        DeclareLaunchArgument(
            "posegraph_file",
            default_value=default_posegraph,
            description=(
                "Fallback posegraph basename for slam_toolbox localization. This "
                "is used when map_name is left empty."
            ),
        ),
        DeclareLaunchArgument(
            "start_at_dock",
            default_value="true",
            description=(
                "Start slam_toolbox localization from the first node in the saved "
                "pose graph."
            ),
        ),
        LogInfo(
            msg=[
                "Starting offboard navigation with slam_toolbox localization map_name=",
                map_name,
                ", posegraph=",
                posegraph_from_map_name,
            ]
        ),
        LogInfo(
            msg=(
                "Nav2 consumes the live /map topic from slam_toolbox localization. "
                "The occupancy map YAML remains available in offboard_slam/maps if "
                "you later want a map_server-based workflow."
            )
        ),
        slam_localization,
        slam_configure,
        slam_activate,
        nav2,
        rviz,
    ])
