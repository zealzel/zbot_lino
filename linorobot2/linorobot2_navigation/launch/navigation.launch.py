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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def include_launch_description(launch_path, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path), launch_arguments=kwargs.items()
    )


def generate_launch_description():
    package_name = "linorobot2_navigation"
    MAP_NAME = "playground"
    default_map_path = get_path(package_name, ["maps", f"{MAP_NAME}.yaml"])
    params_file_path = get_path(package_name, ["config", "navigation.yaml"])
    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    rviz_config_path = get_path("nav2_bringup", ["rviz", "nav2_default_view.rviz"])

    nav2_bringup = include_launch_description(nav2_launch_path, launch_arguments={
        "map": LaunchConfiguration("map"),
        "use_sim_time": LaunchConfiguration("sim"),
        "params_file": LaunchConfiguration("params_file"),
    })
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("sim")}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file_path,
                description=(
                    "Full path to the ROS2 parameters file to use for all launched nodes"
                ),
            ),
            DeclareLaunchArgument(
                name="sim",
                default_value="false",
                description="Enable use_sime_time to true",
            ),
            DeclareLaunchArgument(
                name="map",
                default_value=default_map_path,
                description="Navigation map path",
            ),
            DeclareLaunchArgument(
                name="rviz", default_value="false", description="Run rviz"
            ),
            nav2_bringup,
            rviz,
        ]
    )
