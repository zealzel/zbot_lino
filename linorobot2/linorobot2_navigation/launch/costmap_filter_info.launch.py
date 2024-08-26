#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    package_name = "linorobot2_navigation"
    robot = "lino2"
    # Create our own temporary YAML files that include substitutions
    # lifecycle_nodes = ["filter_mask_server", "costmap_filter_info_server"]
    lifecycle_nodes = ["costmap_filter_info_server"]

    # Parameters
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    mask_yaml_file = LaunchConfiguration("mask")

    # Declare the launch arguments
    worldname_arg = DeclareLaunchArgument(name="worldname", description="worldname")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "keepout_params_file",
        description="Full path to the ROS2 parameters file to use",
        default_value=get_path(package_name, ["params", "keepout_params.yaml"]),
    )

    #
    # declare_mask_yaml_file_cmd = DeclareLaunchArgument(
    #     "mask", description="Full path to filter mask yaml file to load"
    # )
    #
    # Make re-written yaml
    # param_substitutions = {
    #     "topic_name": (LaunchConfiguration("worldname"), "/", robot, "/keepout_filter_mask"),
    # }

    keepout_params_file = LaunchConfiguration("keepout_params_file")
    keepout_params_file = RewrittenYaml(
        source_file=keepout_params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True,
    )

    # Nodes launching commands
    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap_filters",
        namespace=namespace,
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    # start_map_server_cmd = Node(
    #     package="nav2_map_server",
    #     executable="map_server",
    #     name="filter_mask_server",
    #     namespace=namespace,
    #     output="screen",
    #     emulate_tty=True,  # https://github.com/ros2/launch/issues/188
    #     parameters=[configured_params],
    # )

    start_costmap_filter_info_server_cmd = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        namespace=namespace,
        output="screen",
        # emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[keepout_params_file],
        # parameters=[configured_params],
    )

    ld = LaunchDescription()

    ld.add_action(worldname_arg)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_mask_yaml_file_cmd)

    ld.add_action(start_lifecycle_manager_cmd)
    # ld.add_action(start_map_server_cmd)
    ld.add_action(start_costmap_filter_info_server_cmd)

    return ld
