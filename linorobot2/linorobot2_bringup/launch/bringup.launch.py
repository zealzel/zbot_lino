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
    sensors_launch_path = get_path("linorobot2_bringup", ["launch", "sensors.launch.py"])
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
