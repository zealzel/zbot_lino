from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def include_launch_description(launch_path, **kwargs):
    launch_args = kwargs.pop("launch_arguments", {})
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=launch_args.items(),
        **kwargs,
    )


def generate_launch_description():
    MAP_NAME = "turtlebot3_world"  # "playground"
    package_name = "linorobot2_navigation"

    default_map_path = get_path(package_name, ["maps", f"{MAP_NAME}.yaml"])
    params_file_path = get_path(package_name, ["config", "navigation.yaml"])
    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])

    nav2_bringup = include_launch_description(
        nav2_launch_path,
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
        },
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
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file_path,
                description="nav2 params file path",
            ),
            DeclareLaunchArgument(
                name="sim",
                default_value="false",
                description="Enable use_sim_time to true",
            ),
            DeclareLaunchArgument(
                name="map",
                default_value=default_map_path,
                description="Navigation map path",
            ),
            DeclareLaunchArgument(
                name="rviz", default_value="false", description="Run rviz"
            ),
            nav2_bringup,
            rviz,
        ]
    )
