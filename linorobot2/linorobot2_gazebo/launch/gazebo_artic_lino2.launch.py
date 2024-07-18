import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)

def generate_artic_ld_list():
    arrNodes = []
    robot = {
        "name": "artic",
        "x_pos": 0.0,
        "y_pos": 0.5,
        "z_pos": 0.01,
    }
    namespace = f"/{robot['name']}" if robot["name"] else ""
    rsp_launch_path = get_path("articubot_one", ["launch", "rsp.launch.py"])
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={
            "namespace":namespace,
            "use_sim_time": "true",
            "use_ros2_control": "true",
        }.items(),
    )
    arrNodes.append(rsp)

    twist_mux_params = get_path("articubot_one", ["config", "twist_mux.yaml"])
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace=namespace,
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("cmd_vel_out", "diff_cont/cmd_vel_unstamped")],
    )
    arrNodes.append(twist_mux)

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=namespace,
        arguments=[
            "-topic", f"{namespace}/robot_description",
            "-entity", robot['name'],
            "-robot_namespace", namespace,
            "-x", str(robot['x_pos']),
            "-y", str(robot['y_pos']),
        ],
        output="screen",
    )
    arrNodes.append(spawn_entity)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["diff_cont", "--controller-manager-timeout", "10"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_broad"],
    )

    spawner = LaunchDescription([
        TimerAction(
            period=4.0,
            actions=[diff_drive_spawner, joint_broad_spawner],
        )],
    )
    arrNodes.append(spawner)

    return arrNodes

def generate_lino2_ld_list():
    arrNodes = []
    robot = {
        "name": "lino2",
        "x_pos": 0.0,
        "y_pos": -0.5,
        "z_pos": 0.01,
    }
    namespace = f"/{robot['name']}" if robot["name"] else ""
    description_launch_path = get_path("linorobot2_description", ["launch", "description.launch.py"])
    ekf_config_path = get_path("linorobot2_base", ["config", "ekf.yaml"])
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": "True",
            "publish_joints": "false",
        }.items(),
    )
    arrNodes.append(description)

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=namespace,
        arguments=[
            "-topic", f"{namespace}/robot_description",
            "-entity", robot["name"],
            "-robot_namespace", robot["name"],
            "-x", str(robot["x_pos"]),
            "-y", str(robot["y_pos"]),
        ],
        output="screen",
    )
    arrNodes.append(spawn_entity)

    command_timeout = Node(
        package="linorobot2_gazebo",
        executable="command_timeout.py",
        name="command_timeout",
        namespace=namespace,
    )
    arrNodes.append(command_timeout)

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": "True"}, ekf_config_path],
        # remappings=[("odometry/filtered", "odom")],
        remappings=[
            ("odometry/filtered", "odom"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )
    arrNodes.append(robot_localization)

    return arrNodes

def generate_launch_description():
    arrNodes = []

    world_path = get_path("turtlebot3_gazebo", ["worlds", "turtlebot3_world.world"])
    gazebo_launch_path = get_path("gazebo_ros", ["launch", "gazebo.launch.py"])
    
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Gazebo world",
    )
    arrNodes.append(world_arg)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            "world": LaunchConfiguration("world"),
        }.items(),
    )
    arrNodes.append(gazebo)

    arrNodes = arrNodes+generate_artic_ld_list()+generate_lino2_ld_list()

    ld = LaunchDescription()
    for node in arrNodes:
        ld.add_action(node)

    return ld
