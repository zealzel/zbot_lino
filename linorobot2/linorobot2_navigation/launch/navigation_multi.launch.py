import os
from ament_index_python.packages import get_package_share_directory
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
    print("OKEVINLEE!!!")
    package_name = "linorobot2_navigation"

    # simulation only: 2wd|4wd|macanum|zbotlinolong
    # real robot: zbotlino(use rplidar)|zbotlinosick1
    # robot_base = os.getenv('LINOROBOT2_BASE', 'zbotlino')
    # robot_base = os.getenv('LINOROBOT2_BASE', 'zbotlinosick2')
    robot_base = os.getenv("LINOROBOT2_BASE", "zbotlino2")

    # the footprint of both is the same
    if robot_base in ["zbotlino", "zbotlinosick1"]:
        robot_base = "zbotlino"

    # default_map_path = get_path("fitrobot", ["maps", "office_res002_0914.yaml"])
    default_map_path = get_path("fitrobot", ["maps", "lino2_office_20240129.yaml"])
    default_map_path_sim = get_path(package_name, ["maps", "turtlebot3_world.yaml"])
    params_file_path = get_path(
        package_name, ["config", robot_base, "navigation.multiworked.yaml"]
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )
    use_namespace_arg = DeclareLaunchArgument(
        name="use_namespace",
        default_value="false",
        description="Enable use_sime_time to true",
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
        default_value=get_path(package_name, ["rviz", "multi_nav2_default_view.rviz"]),
        # default_value=get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"]),
        description=("Full path to the ROS2 rviz config file"),
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
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": default_map_path_sim,
            },
        ],
        remappings=remappings,
    )
    map_server_lifecyle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("sim")},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    nav_launch_dir = os.path.join(
        get_package_share_directory(package_name), "launch", "nav2_bringup"
    )
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            # "map": LaunchConfiguration("map"),
            "map": "",
            "map_server": "False",
            "namespace": LaunchConfiguration("namespace"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": LaunchConfiguration("params_file"),
            # "default_bt_xml_filename": os.path.join(
            #     get_package_share_directory("nav2_bt_navigator"),
            #     "behavior_trees",
            #     "navigate_w_replanning_and_recovery.xml",
            # ),
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
        namespace=LaunchConfiguration("namespace"),
        output="screen",
    )
    print("AAAA")
    print(LaunchConfiguration("namespace"))
    print("BBB")
    return LaunchDescription(
        [
            namespace_arg,
            use_namespace_arg,
            use_sim_arg,
            use_rviz_arg,
            rviz_config_arg,
            params_arg,
            map_arg,
            map_sim_arg,
            map_server,
            map_server_lifecyle,
            nav2_bringup,
            rviz,
            uros_repub,
        ]
    )
