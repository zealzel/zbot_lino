import os
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
    get_package_share_directory("apriltag_ros"), "cfg", "tags_36h11_filter.yaml"
)


def generate_launch_description():
    package_name = "linorobot2_navigation"
    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )
    # image_remap_input_arg = DeclareLaunchArgument(
    #     name="image_remap",
    #     default_value="camera/color/image_raw",
    #     description="image input topic",
    # )
    # camera_info_remap_input_arg = DeclareLaunchArgument(
    #     name="camera_info_remap",
    #     default_value="camera/color/camera_info",
    #     description="camera info input topic",
    # )
    composable_nodes = [
        ComposableNode(
            package="image_proc",
            plugin="image_proc::RectifyNode",
            name="rectify_node",
            namespace=LaunchConfiguration("namespace"),
            remappings=[
                ("image", "camera/color/image_raw"),
                ("camera_info", "camera/color/camera_info"),
                ("image_rect", "camera/color/image_rect"),
                ("image_rect/compressed", "camera/color/image_rect/compressed"),
                (
                    "image_rect/compressedDepth",
                    "camera/color/image_rect/compressedDepth",
                ),
                ("image_rect/theora", "camera/color/image_rect/theora"),
            ],
        ),
        ComposableNode(
            package="apriltag_ros",
            plugin="AprilTagNode",
            name="apriltag",
            namespace=LaunchConfiguration("namespace"),
            parameters=[config],
            remappings=[
                ("image", LaunchConfiguration("image_remap")),
                ("camera_info", LaunchConfiguration("camera_info_remap")),
                # ("image", "camera/color/image_raw"),
                # ("camera_info", "camera/color/camera_info"),
                # ("/image", "camera/color/image_rect"),
            ],
        ),
    ]
    container = ComposableNodeContainer(
        name="image_proc_container",
        # namespace="",
        namespace=LaunchConfiguration("namespace"),
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
    )
    dock_pose_publisher = Node(
        package=package_name,
        executable="detect_pose",
        namespace=LaunchConfiguration("namespace"),
        name="detect_pose_node",
        parameters=[{"dock_tag_ids": [10, 20, 30]}],
        # parameters=[{"dock_tag_ids": [30]}],
    )
    return LaunchDescription(
        [
            namespace_arg,
            container,
            dock_pose_publisher,
        ]
    )
