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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "playground.world"]
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "launch", "description.launch.py"]
    )
    x_arg = DeclareLaunchArgument("x", default_value="0", description="x position")
    y_arg = DeclareLaunchArgument("y", default_value="0", description="y position")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Gazebo world",
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ]
            )
        ),
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
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "use_sim_time": str(use_sim_time),
            "publish_joints": "false",
        }.items(),
    )
    return LaunchDescription(
        [
            x_arg,
            y_arg,
            world_arg,
            gazebo,
            spawn_entity,
            command_timeout,
            robot_localization,
            launch_description,
        ]
    )
