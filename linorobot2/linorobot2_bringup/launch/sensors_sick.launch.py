import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    laser_sensor_name = os.getenv("LINOROBOT2_LASER_SENSOR", "nanoscan3")
    depth_sensor_name = os.getenv("LINOROBOT2_DEPTH_SENSOR", "")
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )
    # namespace = "/lino2"
    namespace = LaunchConfiguration("namespace")

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_bringup"), "launch", "lasers.launch.py"]
    )

    depth_launch_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_bringup"), "launch", "depth.launch.py"]
    )

    return LaunchDescription(
        [
            namespace_arg,
            # front lidar laser
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(laser_launch_path),
                launch_arguments={
                    "node_name": "sick_safetyscanners2_node_1",
                    "sensor": laser_sensor_name,
                    "sensor_ip": "192.168.1.2",
                    "host_ip": "192.168.1.3",
                    # 'topic_name': 'scan1',
                    # "topic_name": f"{namespace}/scan1",
                    "topic_name": [namespace, "scan1"],
                    "frame_id": "laser1",
                    "namespace": namespace,
                }.items(),
            ),
            # back lidar laser
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(laser_launch_path),
                launch_arguments={
                    "node_name": "sick_safetyscanners2_node_2",
                    "sensor": laser_sensor_name,
                    "sensor_ip": "192.168.1.4",
                    "host_ip": "192.168.1.12",
                    # "topic_name": "scan2",
                    # "topic_name": f"{namespace}/scan2",
                    "topic_name": [namespace, "scan2"],
                    "frame_id": "laser2",
                    "namespace": namespace,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(depth_launch_path),
                condition=IfCondition(
                    PythonExpression(['"" != "', depth_sensor_name, '"'])
                ),
                launch_arguments={"sensor": depth_sensor_name}.items(),
            ),
        ]
    )
