# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    robots = [
        {
            "name": "robot1",
            "x_pos": 0.0,
            "y_pos": 0.5,
            "z_pos": 0.01,
            "color_name": "Yellow",
            "color_rgb": "1 1 0 1",
        },
        {
            "name": "robot2",
            "x_pos": 0.0,
            "y_pos": -0.5,
            "z_pos": 0.01,
            "color_name": "Blue",
            "color_rgb": "0 0 1 1",
        },
    ]

    arrNodes = []

    use_sim_time = True

    # world_path = get_path("linorobot2_gazebo", ["worlds", f"playground.world"])
    world_path = get_path("turtlebot3_gazebo", ["worlds", "turtlebot3_world.world"])

    description_launch_path = get_path(
        "linorobot2_description", ["launch", "description.launch.py"]
    )
    gazebo_launch_path = get_path("gazebo_ros", ["launch", "gazebo.launch.py"])
    ekf_config_path = get_path("linorobot2_base", ["config", "ekf.yaml"])

    # x_arg = DeclareLaunchArgument("x", default_value="0.5", description="x position")
    # y_arg = DeclareLaunchArgument("y", default_value="0.5", description="y position")
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

    #
    # arrNodes.append(map_server)
    # arrNodes.append(map_server_lifecyle)
    #
    for robot in robots:
        namespace = f"/{robot['name']}" if robot["name"] else ""

        description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": str(use_sim_time),
                "publish_joints": "false",
            }.items(),
        )
        arrNodes.append(description)

        spawn_entity = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            namespace=namespace,
            arguments=[
                "-topic",
                f"{namespace}/robot_description",
                "-entity",
                robot["name"],
                "-robot_namespace",
                robot["name"],
                "-x",
                str(robot["x_pos"]),
                "-y",
                str(robot["y_pos"]),
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
            parameters=[{"use_sim_time": use_sim_time}, ekf_config_path],
            # remappings=[("odometry/filtered", "odom")],
            remappings=[
                ("odometry/filtered", "odom"),
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        )
        arrNodes.append(robot_localization)

    ld = LaunchDescription()
    for node in arrNodes:
        ld.add_action(node)

    return ld
