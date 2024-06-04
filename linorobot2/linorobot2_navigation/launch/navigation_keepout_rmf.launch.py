import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
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


def include_launch_description(launch_path, **kwargs):
    launch_args = kwargs.pop("launch_arguments", {})
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=launch_args.items(),
        **kwargs,
    )


def generate_launch_description():
    package_name = "linorobot2_navigation"
    MAP_NAME = "fit_office_res002_0926"

    # simulation only: 2wd|4wd|macanum|zbotlinolong|zbotlinosick1|zbotlinosick2
    robot_base = os.getenv('LINOROBOT2_BASE', 'zbotlino')

    # the footprint of both is the same
    if robot_base == ["zbotlino", "zbotlinosick1"]:
        robot_base = "zbotlino"

    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    costmap_filter_info_launch_path = get_path(package_name, ["launch", "costmap_filter_info.launch.py"])
    rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])

    sim_arg = DeclareLaunchArgument(name="sim", default_value="true")
    rviz_arg = DeclareLaunchArgument(name="rviz", default_value="true")

    map_name_arg = DeclareLaunchArgument("map_name", default_value=MAP_NAME)

    maploc = os.path.join(get_package_share_directory(package_name), 'maps')
    map_arg = DeclareLaunchArgument(
        name="map",
        default_value=[f'{maploc}/', LaunchConfiguration("map_name"), ".yaml"],
        description="Navigation map path",
    )

    maskloc = os.path.join(get_package_share_directory(package_name), 'masks')
    mask_arg = DeclareLaunchArgument(
        "mask",
        default_value=[f'{maskloc}/', "keepout_mask_", LaunchConfiguration("map_name"), ".yaml"],
        description="mask file for keepout layer",
    )

    params_file_path = get_path(package_name, ["config", robot_base, "navigation_keepout.yaml"])
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file_path,
        description="nav2 params file path",
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
        }.items()
    )

    # if not delayed, nav2_bringup will not able to launch controllers successfully
    costmap_filter_info_delayed = LaunchDescription([
        TimerAction(
            period=4.0,
            actions=[costmap_filter_info],
        )],
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
    return LaunchDescription(
        [
            sim_arg,
            rviz_arg,
            map_name_arg,
            map_arg,
            mask_arg,
            params_file_arg,
            keepout_params_arg,
            nav2_bringup,
            costmap_filter_info_delayed,
            rviz,
        ]
    )
