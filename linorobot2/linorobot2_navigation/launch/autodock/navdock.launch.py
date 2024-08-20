from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def get_path(package_name, subpaths=None):
    if subpaths:
        return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)
    else:
        return PathJoinSubstitution([FindPackageShare(package_name)])


def generate_launch_description():
    package_name = "linorobot2_navigation"
    use_sim_arg = DeclareLaunchArgument(
        name="sim",
        default_value="false",
        description="Enable use_sime_time to true",
    )
    use_rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="true", description="Run rviz"
    )
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )
    # rviz_config_path = get_path(package_name, ["rviz", "nav2_camera.rviz"])
    rviz_config_path = get_path(package_name, ["rviz", "multi_nav2_range_dock.rviz"])
    lino_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_path("linorobot2_navigation", ["launch", "nav.launch.py"])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "sim": LaunchConfiguration("sim"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": rviz_config_path,
        }.items(),
    )
    dock_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_path(package_name, ["launch", "autodock", "dock.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "namespace": LaunchConfiguration("namespace"),
        }.items(),
    )
    return LaunchDescription(
        [
            use_sim_arg,
            use_rviz_arg,
            namespace_arg,
            lino_navigation,
            dock_robot,
        ]
    )
