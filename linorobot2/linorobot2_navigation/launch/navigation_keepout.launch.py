import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)


def get_path(package_name, subpaths):
    package_share_directory = PathJoinSubstitution(
        [FindPackageShare(package_name)] + subpaths
    )
    return package_share_directory


def generate_launch_description():
    package_name = "linorobot2_navigation"

    # simulation only: 2wd|4wd|macanum|zbotlinolong
    # real robot: zbotlino(use rplidar)|zbotlinosick1
    robot_base = os.getenv("LINOROBOT2_BASE", "zbotlino")

    # the footprint of both is the same
    if robot_base in ["zbotlino", "zbotlinosick1"]:
        robot_base = "zbotlino"

    default_map_path = get_path("fitrobot", ["maps", "office_res002_0914.yaml"])
    #default_map_path = get_path("fitrobot", ["maps", "office_res002_0523.yaml"])
    default_map_path_sim = get_path(package_name, ["maps", "turtlebot3_world.yaml"])

    default_mask_path = get_path(
        #"fitrobot", ["masks", "keepout_mask_office_res002_0523.yaml"]
        "fitrobot", ["masks", "keepout_mask_office_res002_0914.yaml"]
    )
    default_mask_path_sim = get_path(
        package_name, ["masks", "keepout_mask_turtlebot3_world.yaml"]
    )
    default_params_file_path = get_path(
        package_name, ["config", robot_base, "navigation_keepout.yaml"]
    )
    costmap_filter_info_launch_path = get_path(
        package_name, ["launch", "costmap_filter_info.launch.py"]
    )
    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])

    use_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="false",
        description="Enable use_sime_time to true",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="false", description="Run rviz"
    )
    map_arg = DeclareLaunchArgument(
        name="map",
        default_value=default_map_path,
        description="Navigation map path",
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )
    map_sim_arg = DeclareLaunchArgument(
        name="map",
        default_value=default_map_path_sim,
        description="Navigation map path",
        condition=IfCondition(LaunchConfiguration("sim")),
    )
    mask_arg = DeclareLaunchArgument(
        "mask",
        default_value=default_mask_path,
        description="mask file for keepout layer",
        condition=UnlessCondition(LaunchConfiguration("sim")),
    )
    mask_sim_arg = DeclareLaunchArgument(
        "mask",
        default_value=default_mask_path_sim,
        description="mask file for keepout layer",
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file_path,
        description=(
            "Full path to the ROS2 parameters file to use for all launched nodes"
        ),
    )
    keepout_params_arg = DeclareLaunchArgument(
        "keepout_params_file",
        default_value=get_path(package_name, ["params", "keepout_params.yaml"]),
        description="params file for keepout layer",
    )

    costmap_filter_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(costmap_filter_info_launch_path),
        launch_arguments={
            "params_file": LaunchConfiguration("keepout_params_file"),
            "mask": LaunchConfiguration("mask"),
        }.items(),
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )
    # if not delayed, nav2_bringup will not able to launch controllers successfully
    costmap_filter_info_delayed = LaunchDescription(
        [
            TimerAction(
                period=4.0,
                actions=[costmap_filter_info],
            )
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("sim")}],
    )
    # Temporary node to republish the range1/data & range2/data to range1_sensor & range2_sensor
    uros_repub = Node(
        package=package_name,
        executable="repub_node",
        name="repub_node",
        output="screen",
    )
    return LaunchDescription(
        [
            use_sim_arg,
            use_rviz_arg,
            keepout_params_arg,
            map_arg, map_sim_arg,
            mask_arg, mask_sim_arg,
            params_arg,
            nav2_bringup,
            costmap_filter_info_delayed,
            rviz,
            uros_repub,
        ]
    )
