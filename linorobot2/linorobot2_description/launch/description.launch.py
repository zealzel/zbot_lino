import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
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

    # simulation only: 2wd|4wd|macanum|zbotlinolong
    # real robot: zbotlino(use rplidar)|zbotlinosick1|zbotlinosick2|zbotlino2
    # robot_base = os.getenv("LINOROBOT2_BASE", "zbotlinosick2")
    robot_base = os.getenv("LINOROBOT2_BASE", "zbotlino2")

    urdf_path = get_path(
        "linorobot2_description", ["urdf", "robots", f"{robot_base}.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "rviz", "description.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="namespace",
                default_value="",
                description="Namespace",
            ),
            DeclareLaunchArgument(
                name="urdf", default_value=urdf_path, description="URDF path"
            ),
            DeclareLaunchArgument(
                name="publish_joints",
                default_value="true",
                description="Launch joint_states_publisher",
            ),
            DeclareLaunchArgument(
                name="rviz", default_value="false", description="Run rviz"
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                namespace=LaunchConfiguration("namespace"),
                condition=IfCondition(LaunchConfiguration("publish_joints")),
                # parameters=[
                #     {'use_sim_time': LaunchConfiguration('use_sim_time')}
                # ] #since galactic use_sim_time gets passed somewhere and rejects this when defined from launch file
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": Command(
                            ["xacro ", LaunchConfiguration("urdf")]
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                condition=IfCondition(LaunchConfiguration("rviz")),
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
