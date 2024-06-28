import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
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
    package_name = "linorobot2_navigation"

    # simulation only: 2wd|4wd|macanum|zbotlinolong
    # real robot: zbotlino(use rplidar)|zbotlinosick1
    # robot_base = os.getenv('LINOROBOT2_BASE', 'zbotlino')
    # robot_base = os.getenv('LINOROBOT2_BASE', 'zbotlinosick2')
    robot_base = os.getenv('LINOROBOT2_BASE', 'zbotlino2')
    package_name = "linorobot2_navigation"

    # the footprint of both is the same
    if robot_base in ["zbotlino", "zbotlinosick1"]:
        robot_base = "zbotlino"

    #default_map_path = get_path("fitrobot", ["maps", "office_res002_0914.yaml"])
    default_map_path = get_path("fitrobot", ["maps", "lino2_office_20240129.yaml"])

    default_map_path_sim = get_path(package_name, ["maps", "turtlebot3_world.yaml"])

    params_file_path = get_path(package_name, ["config", robot_base, "navigation.yaml"])
    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    # rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])
    # rviz_config_path = "/home/zealzel/.rviz2/nav2_camera.rviz"

    use_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="false",
        description="Enable use_sime_time to true",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="false", description="Run rviz"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"]),
        description=(
            "Full path to the ROS2 rviz config file"
        ),
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file_path,
        description=(
            "Full path to the ROS2 parameters file to use for all launched nodes"
        ),
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
        # arguments=["-d", rviz_config_path],
        arguments=["-d", LaunchConfiguration("rviz_config")],
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
            rviz_config_arg,
            params_arg,
            map_arg, map_sim_arg,
            nav2_bringup,
            rviz,
            uros_repub,
        ]
    )
