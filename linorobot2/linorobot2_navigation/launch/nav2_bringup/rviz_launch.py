import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def generate_launch_description():
    robot = "lino2"

    robots_env = os.getenv("ROBOT_INFO", "")
    if not robots_env:
        print("ROBOT_INFO env is not set")
        print("example: ROBOT_INFO=lino2:13a5")
        print("  robot is defined by 2 arguments which is separated by :")
        print("    arg1: robot_type\n    arg2: robot_sn")
        return LaunchDescription([])

    robot_first = [e.split(":") for e in robots_env.split(";")][0]
    robot_type, robot_sn = robot_first[0], robot_first[1]

    def worldname_namespace_to_rviz(context):
        worldname = context.launch_configurations.get("worldname", "")
        # namespace = context.launch_configurations.get("namespace", "")

        # namespace = f"/{robot_type}_{robot_sn}"
        namespace = f"/{robot_type}_{robot_sn}" if robot_type and robot_sn else ""

        rviz_config_file = context.launch_configurations.get(
            "rviz_config",
            os.path.join(
                get_package_share_directory("linorobot2_navigation"),
                "rviz",
                "multi_nav2_range.rviz",
            ),
        )
        slam_rviz_config_file = os.path.join(
            get_package_share_directory("linorobot2_navigation"),
            "rviz",
            "slam.rviz",
        )
        namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={
                "<map_topic>": f"/{worldname}/{robot}/map",
                "<map_updates_topic>": f"/{worldname}/{robot}/map_updates",
            },
        )
        # if namespace and worldname:
        if worldname:
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                namespace=namespace,
                arguments=["-d", namespaced_rviz_config_file],
                output="screen",
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/goal_pose", "goal_pose"),
                    ("/clicked_point", "clicked_point"),
                    ("/initialpose", "initialpose"),
                ],
            )
        elif namespace and not worldname:
            rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                namespace=namespace,
                arguments=["-d", slam_rviz_config_file],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            )

        return [
            rviz_node,
        ]

    rviz_cmd = OpaqueFunction(function=worldname_namespace_to_rviz)
    ld = LaunchDescription()
    ld.add_action(rviz_cmd)

    return ld
