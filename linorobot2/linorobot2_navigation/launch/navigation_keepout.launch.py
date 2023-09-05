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

MAP_NAME = "playground"  # change to the name of your own map here


def get_path(package_name, subpaths):
    package_share_directory = PathJoinSubstitution(
        [FindPackageShare(package_name)] + subpaths
    )
    return package_share_directory


def generate_launch_description():
    package_name = "linorobot2_navigation"
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=get_path(package_name, ["config", "navigation_keepout.yaml"]),
        description=(
            "Full path to the ROS2 parameters file to use for all launched nodes"
        ),
    )
    keepout_params_arg = DeclareLaunchArgument(
        "keepout_params_file",
        default_value=get_path(package_name, ["params", "keepout_params.yaml"]),
        description="params file for keepout layer",
    )
    mask_arg = DeclareLaunchArgument(
        "mask",
        description="mask file for keepout layer",
    )
    nav2_launch_path = get_path("nav2_bringup", ["launch", "bringup_launch.py"])
    costmap_filter_info_launch_path = get_path(package_name, ["launch", "costmap_filter_info.launch.py"])
    rviz_config_path = get_path(package_name, ["rviz", "linorobot2_navigation.rviz"])
    default_map_path = get_path(package_name, ["maps", f"{MAP_NAME}.yaml"])

    use_sim_arg = (
        DeclareLaunchArgument(
            name="sim",
            default_value="false",
            description="Enable use_sime_time to true",
        ),
    )
    use_rviz_arg = (
        DeclareLaunchArgument(
            name="rviz", default_value="false", description="Run rviz"
        ),
    )
    map_arg = (
        DeclareLaunchArgument(
            name="map",
            default_value=default_map_path,
            description="Navigation map path",
        ),
    )

    # ros2 launch nav2_costmap_filters_demo costmap_filter_info.launch.py \
    # params_file:=$HOME/simulations/src/navigation2_tutorials/nav2_costmap_filters_demo/params/keepout_params.yaml \
    # mask:=$HOME/maps/keepout_mask.yaml

    costmap_filter_info = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(costmap_filter_info_launch_path),
            launch_arguments={
                "params_file": LaunchConfiguration("keepout_params_file"),
                "mask": LaunchConfiguration("mask"),
            }.items(),
        ),
    )
    nav2_bringup = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": LaunchConfiguration("sim"),
                "params_file": LaunchConfiguration("params_file"),
            }.items(),
        ),
    )
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
            use_sim_arg,
            use_rviz_arg,
            keepout_params_arg,
            mask_arg,
            map_arg,
            params_arg,
            costmap_filter_info,
            nav2_bringup,
            rviz,
        ]
    )
