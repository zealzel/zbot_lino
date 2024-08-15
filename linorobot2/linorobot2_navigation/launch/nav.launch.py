import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "linorobot2_navigation"
    fitrobot_install_path = get_package_share_directory("fitrobot")

    # simulation only: 2wd|4wd|macanum|zbotlinolong
    # real robot: zbotlino(use rplidar)|zbotlinosick1
    # robot_base = os.getenv('LINOROBOT2_BASE', 'zbotlinosick2')

    robot_base = os.getenv("LINOROBOT2_BASE", "zbotlino2")
    package_name = "linorobot2_navigation"

    # the footprint of both is the same
    if robot_base in ["zbotlino", "zbotlinosick1"]:
        robot_base = "zbotlino"

    params_file_path = get_path(
        # package_name, ["config", robot_base, "navigation.multiworked.yaml"]
        package_name,
        ["config", robot_base, "navigation_keepout.yaml"],
    )
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
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
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file_path,
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
            "use_namespace": "True",
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, "rviz_launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("sim"),
            "namespace": LaunchConfiguration("namespace"),
            "use_namespace": "True",
            "rviz_config": LaunchConfiguration("rviz_config"),
            "log_level": "warn",
        }.items(),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    uros_repub = Node(
        package=package_name,
        executable="repub_node",
        name="repub_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        remappings=[
            ("/range1/data", "range1/data"),
            ("/range2/data", "range2/data"),
            ("/range3/data", "range3/data"),
            ("/range4/data", "range4/data"),
            ("/range1_sensor", "local_costmap/range1_sensor"),
            ("/range2_sensor", "local_costmap/range2_sensor"),
            ("/range3_sensor", "local_costmap/range3_sensor"),
            ("/range4_sensor", "local_costmap/range4_sensor"),
        ],
    )
    # Temporary node to republish the range1/data & range2/data to range1_sensor & range2_sensor
    return LaunchDescription(
        [
            namespace_arg,
            use_sim_arg,
            use_rviz_arg,
            rviz_config_arg,
            params_arg,
            nav2_bringup,
            rviz,
            uros_repub,
        ]
    )
