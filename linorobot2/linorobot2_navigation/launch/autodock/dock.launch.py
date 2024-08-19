import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "linorobot2_navigation"

    detect_pose_launch_dir = os.path.join(
        get_package_share_directory(package_name), "launch", "autodock"
    )
    dock_params_dir = os.path.join(get_package_share_directory(package_name), "params")
    params_file = os.path.join(dock_params_dir, "my_docking_backward.yaml")

    with open(params_file, "r") as file:
        params = yaml.safe_load(file)

    # 動態修改參數
    # dock_params = params['docking_server']['ros__parameters']['my_dock_forward']
    dock_params = params["docking_server"]["ros__parameters"]["my_dock"]

    # 保留 my_dock_yaw1 和 my_dock_yaw2 的 staging_yaw_offset
    my_dock_yaw1_staging_yaw_offset = params["docking_server"]["ros__parameters"][
        "my_dock_yaw1"
    ]["staging_yaw_offset"]
    my_dock_yaw2_staging_yaw_offset = params["docking_server"]["ros__parameters"][
        "my_dock_yaw2"
    ]["staging_yaw_offset"]
    my_dock_yaw3_staging_yaw_offset = params["docking_server"]["ros__parameters"][
        "my_dock_yaw3"
    ]["staging_yaw_offset"]

    params["docking_server"]["ros__parameters"]["my_dock_yaw1"] = dock_params.copy()
    params["docking_server"]["ros__parameters"]["my_dock_yaw2"] = dock_params.copy()
    params["docking_server"]["ros__parameters"]["my_dock_yaw3"] = dock_params.copy()

    # 恢復 staging_yaw_offset
    params["docking_server"]["ros__parameters"]["my_dock_yaw1"][
        "staging_yaw_offset"
    ] = my_dock_yaw1_staging_yaw_offset
    params["docking_server"]["ros__parameters"]["my_dock_yaw2"][
        "staging_yaw_offset"
    ] = my_dock_yaw2_staging_yaw_offset
    params["docking_server"]["ros__parameters"]["my_dock_yaw3"][
        "staging_yaw_offset"
    ] = my_dock_yaw3_staging_yaw_offset

    dock_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                detect_pose_launch_dir,
                "detect.launch.py",
            )
        )
    )
    docking_server = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        output="screen",
        # parameters=[params_file],
        # parameters=[params],
        parameters=[params["docking_server"]["ros__parameters"]],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_docking",
        output="screen",
        parameters=[{"autostart": True}, {"node_names": ["docking_server"]}],
    )
    return LaunchDescription(
        [
            dock_detection_launch,
            docking_server,
            lifecycle_manager,
        ]
    )
