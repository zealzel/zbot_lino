import os
from ament_index_python.packages import get_package_share_directory
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

    default_params_file_path = get_path(
        package_name, ["config", robot_base, "navigation_keepout.yaml"]
    )
    costmap_filter_info_launch_path = get_path(
        package_name, ["launch", "costmap_filter_info.launch.py"]
    )

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
        default_value=get_path(package_name, ["rviz", "multi_nav2_range.rviz"]),
        description=("Full path to the ROS2 rviz config file"),
    )
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="Namespace",
    )
    use_namespace_arg = DeclareLaunchArgument(
        name="use_namespace",
        default_value="false",
        description="Enable use_sime_time to true",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file_path,
        description=(
            "Full path to the ROS2 parameters file to use for all launched nodes"
        ),
    )
    
    nav_launch_dir = os.path.join(
        get_package_share_directory(package_name), "launch", "nav2_bringup"
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "map": "",
            "map_server": "False",
            "namespace": LaunchConfiguration("namespace"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, "rviz_launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("sim"),
            "namespace": LaunchConfiguration("namespace"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "log_level": "warn",
        }.items(),
        condition=IfCondition(LaunchConfiguration("rviz")),
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
            namespace_arg,
            use_namespace_arg,
            use_sim_arg,
            use_rviz_arg,
            rviz_config_arg,
            params_arg,
            nav2_bringup,
            rviz,
            uros_repub,
        ]
    )
