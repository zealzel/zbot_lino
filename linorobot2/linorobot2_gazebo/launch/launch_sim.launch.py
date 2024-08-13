import os
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_path(package_name, subpaths=None):
    if subpaths:
        return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)
    else:
        return PathJoinSubstitution([FindPackageShare(package_name)])


# use of opaqueFunction
# 1. https://answers.ros.org/question/404041/ros2-python-launch-using-argument-to-create-file-name-for-a-launch_argument/
# 2. https://www.robotsfan.com/posts/7a5950c4.html

# usages
# ros2 launch linorobot2_gazebo launch_sim.launch.py worldname:=room_with_tags x:=0.5 y:=0.5

WORLD_INFO = {
    "turtlebot3_world": {"ext": "world", "x": 0.5, "y": 0.5},
    "turtlebot3_house": {"ext": "world", "x": -3.0, "y": 1.0},
    "room_with_tags": {"ext": "sdf", "x": 1.0, "y": 1.0},
    "obstacles": {"ext": "world", "x": 0.0, "y": 0.0},
}


def generate_launch_description():
    package_name = "linorobot2_gazebo"
    pkg_install_path = get_package_share_directory(package_name)
    fitrobot_install_path = get_package_share_directory("fitrobot")
    os.path.join(pkg_install_path, "bringup_launch.py")
    default_worldname = "turtlebot3_world"

    if "GAZEBO_RESOURCE_PATH" in os.environ:
        resource_path = fitrobot_install_path + ":" + os.environ["GAZEBO_RESOURCE_PATH"]
    else:
        resource_path = fitrobot_install_path

    def worldname_get(context):
        if "worldpath" in context.launch_configurations:
            if "worldname" in context.launch_configurations:
                raise RuntimeError("world and worldname cannot be set at the same time")
        else:
            worldname = context.launch_configurations.get(
                "worldname", default_worldname
            )
            worldpath = os.path.join(
                fitrobot_install_path,
                "worlds",
                f"{worldname}.{WORLD_INFO[worldname]['ext']}",
            )
            return [SetLaunchConfiguration("world", worldpath)]

    def worldpath_get(context):
        if "worldpath" in context.launch_configurations:
            if "worldname" in context.launch_configurations:
                raise RuntimeError("world and worldname cannot be set at the same time")
            worldpath = context.launch_configurations["worldpath"]
            return [SetLaunchConfiguration("world", worldpath)]

    def x_position(context):
        if "worldpath" in context.launch_configurations:
            if "worldname" in context.launch_configurations:
                raise RuntimeError("world and worldname cannot be set at the same time")
            x = context.launch_configurations.get("x", "0.0")
        else:
            worldname = context.launch_configurations["worldname"]
            x = context.launch_configurations.get("x", str(WORLD_INFO[worldname]["x"]))
        return [SetLaunchConfiguration("x", x)]

    def y_position(context):
        if "worldpath" in context.launch_configurations:
            if "worldname" in context.launch_configurations:
                raise RuntimeError("world and worldname cannot be set at the same time")
            y = context.launch_configurations.get("y", "0.0")
        else:
            worldname = context.launch_configurations["worldname"]
            y = context.launch_configurations.get("y", str(WORLD_INFO[worldname]["y"]))
        return [SetLaunchConfiguration("y", y)]

    use_sim_time = True
    description_launch_path = get_path(
        "linorobot2_description", ["launch", "description.launch.py"]
    )
    gazebo_launch_path = get_path("gazebo_ros", ["launch", "gazebo.launch.py"])
    ekf_config_path = get_path("linorobot2_base", ["config", "ekf.yaml"])

    worldname_arg = OpaqueFunction(function=worldname_get)
    worldpath_arg = OpaqueFunction(function=worldpath_get)
    x_arg = OpaqueFunction(function=x_position)
    y_arg = OpaqueFunction(function=y_position)

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
            "publish_joints": "false",
        }.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "zbot_lino",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
        ],
        output="screen",
    )
    command_timeout = Node(
        package="linorobot2_gazebo",
        executable="command_timeout.py",
        name="command_timeout",
    )
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value=resource_path),
            SetEnvironmentVariable(name="LINOROBOT2_BASE", value="zbotlino2"),
            # SetEnvironmentVariable(name="LINOROBOT2_BASE", value="zbotlino2a"),
            x_arg,
            y_arg,
            worldname_arg,
            worldpath_arg,
            description,
            gazebo,
            spawn_entity,
            command_timeout,
            robot_localization,
        ]
    )
