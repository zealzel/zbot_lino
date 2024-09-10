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

from nav2_common.launch import RewrittenYaml, ReplaceString
from launch.actions import LogInfo


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "linorobot2_navigation"
    # robot_type = "lino2"

    robots_env = os.getenv("ROBOT_INFO", "")
    if not robots_env:
        print("ROBOT_INFO env is not set")
        print("example: ROBOT_INFO=lino2:13a5")
        print("  robot is defined by 2 arguments which is separated by :")
        print("    arg1: robot_type\n    arg2: robot_sn")
        return LaunchDescription([])

    robot_first = [e.split(":") for e in robots_env.split(";")][0]
    robot_type, robot_sn = robot_first[0], robot_first[1]

    # simulation only: 2wd|4wd|macanum|zbotlinolong
    # real robot: zbotlino(use rplidar)|zbotlinosick1
    robot_base = os.getenv("LINOROBOT2_BASE", "zbotlino2")
    package_name = "linorobot2_navigation"

    # the footprint of both is the same
    if robot_base in ["zbotlino", "zbotlinosick1"]:
        robot_base = "zbotlino"

    params_file_path = get_path(package_name, ["config", robot_base, "navigation.yaml"])

    namespace = f"/{robot_type}_{robot_sn}" if robot_type and robot_sn else ""

    worldname_arg = DeclareLaunchArgument(name="worldname", description="worldname")

    use_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="false",
        description="Enable use_sime_time to true",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="false", description="Run rviz"
    )
    use_composition_arg = DeclareLaunchArgument(
        name="use_composition",
        default_value="True",
        description="Enable use composition",
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
    params_file = LaunchConfiguration("params_file")
    params_file = ReplaceString(
        source_file=params_file,
        replacements={
            "<map_topic>": ("/", LaunchConfiguration("worldname"), "/", robot_type, "/map"),
            "<scan1>": f"{namespace}/scan1",
            "<scan2>": f"{namespace}/scan2",
            "<costmap_filter_info>": f"{namespace}/costmap_filter_info",
        },
    )
    rviz_config_file = LaunchConfiguration("rviz_config")
    rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={
            "<map_topic>": ("/", LaunchConfiguration("worldname"), "/", robot_type, "/map"),
            "<map_updates_topic>": ("/", LaunchConfiguration("worldname"), "/", robot_type, "/map_updates"),
        },
    )

    costmap_filter_info_launch_path = get_path(
        package_name, ["launch", "costmap_filter_info.launch.py"]
    )
    keepout_params_arg = DeclareLaunchArgument(
        "keepout_params_file",
        default_value=get_path(package_name, ["params", "keepout_params.yaml"]),
        description="params file for keepout layer",
    )
    keepout_params_file = LaunchConfiguration("keepout_params_file")
    keepout_params_file = ReplaceString(
        source_file=keepout_params_file,
        replacements={
            "keepout_filter_mask": ( "/", LaunchConfiguration("worldname"), "/", robot_type, "/keepout_filter_mask",
            )
        },
    )
    costmap_filter_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(costmap_filter_info_launch_path),
        launch_arguments={
            "namespace": namespace,
            "worldname": LaunchConfiguration("worldname"),
            "keepout_params_file": keepout_params_file,
        }.items(),
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
            "namespace": namespace,
            "use_namespace": "True",
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": params_file,
            "mapkey": (LaunchConfiguration("worldname"), "/", robot_type),
            "use_composition": LaunchConfiguration("use_composition"),
        }.items(),
    )
    print("mapkey: ", f"/{robot_type}")

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, "rviz_launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("sim"),
            "worldname": LaunchConfiguration("worldname"),
            "namespace": namespace,
            "use_namespace": "True",
            "rviz_config": rviz_config_file,
            "log_level": "warn",
        }.items(),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    uros_repub = Node(
        package=package_name,
        executable="repub_node",
        name="repub_node",
        namespace=namespace,
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
            # namespace_arg,
            worldname_arg,
            use_sim_arg,
            use_rviz_arg,
            use_composition_arg,
            rviz_config_arg,
            params_arg,
            keepout_params_arg,
            costmap_filter_info,
            nav2_bringup,
            rviz,
            uros_repub,
            LogInfo(msg=["params_file: ", params_file]),
            LogInfo(msg=["keepout_params_file: ", keepout_params_file]),
        ]
    )
