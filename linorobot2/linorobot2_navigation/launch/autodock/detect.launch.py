import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
    get_package_share_directory("apriltag_ros"), "cfg", "tags_36h11_filter.yaml"
)


def generate_launch_description():
    package_name = "linorobot2_navigation"
    composable_nodes = [
        ComposableNode(
            package="image_proc",
            plugin="image_proc::RectifyNode",
            name="rectify_node",
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
            parameters=[config],
            remappings=[
                ("/image", "camera/color/image_raw"),
                # ("/image", "camera/color/image_rect"),
                ("/camera_info", "camera/color/camera_info"),
            ],
        ),
    ]
    container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
    )
    dock_pose_publisher = Node(
        package=package_name,
        executable="detect_pose",
        name="detect_pose_node",
        parameters=[{"dock_tag_ids": [10, 20, 30]}],
        # parameters=[{"dock_tag_ids": [30]}],
    )
    return LaunchDescription(
        [
            container,
            dock_pose_publisher,
        ]
    )
