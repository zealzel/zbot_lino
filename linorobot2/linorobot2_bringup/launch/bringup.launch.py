import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def include_launch_description(launch_path, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path), launch_arguments=kwargs.items()
    )


def generate_launch_description():
    # real robot: zbotlino(use rplidar)|zbotlinosick1|zbotlinosick2
    robot_base = os.getenv("LINOROBOT2_BASE", "zbotlinosick2")
    if robot_base in ["zbotlinosick2", "zbotlino2"]:
        sensor_launch = "sensors_sick"
    else:
        sensor_launch = "sensors"

    sensors_launch_path = get_path("linorobot2_bringup", ["launch", f"{sensor_launch}.launch.py"])
    description_launch_path = get_path("linorobot2_description", ["launch", "description.launch.py"])
    ekf_config_path = get_path("linorobot2_base", ["config", "ekf.yaml"])
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path],
        remappings=[("odometry/filtered", "odom")],
    )
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", LaunchConfiguration("base_serial_port")],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="base_serial_port",
                default_value="/dev/ttyACM0",
                description="Linorobot Base Serial Port",
            ),
            DeclareLaunchArgument(
                name="joy", default_value="false", description="Use Joystick"
            ),
            ekf,
            include_launch_description(description_launch_path),
            include_launch_description(sensors_launch_path),
            micro_ros_agent,
        ]
    )
