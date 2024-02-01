from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument("node_name")
    frame_id_arg = DeclareLaunchArgument("frame_id")
    topic_name_arg = DeclareLaunchArgument("topic_name")
    sensor_ip_arg = DeclareLaunchArgument("sensor_ip")
    host_ip_arg = DeclareLaunchArgument("host_ip")
    return LaunchDescription(
        [
            node_name_arg,
            frame_id_arg,
            sensor_ip_arg,
            host_ip_arg,
            topic_name_arg,
            Node(
                package="sick_safetyscanners2",
                executable="sick_safetyscanners2_node",
                name=LaunchConfiguration("node_name"),
                # name="sick_safetyscanners2_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "frame_id": LaunchConfiguration("frame_id"),
                        "sensor_ip": LaunchConfiguration("sensor_ip"),
                        "host_ip": LaunchConfiguration("host_ip"),
                        "interface_ip": "0.0.0.0",
                        "host_udp_port": 0,
                        "channel": 0,
                        "channel_enabled": True,
                        "skip": 0,
                        "angle_start": -1.745,
                        "angle_end": 1.745,
                        "time_offset": 0.0,
                        "general_system_state": True,
                        "derived_settings": True,
                        "measurement_data": True,
                        "intrusion_data": True,
                        "application_io_data": True,
                        "use_persistent_config": False,
                        "min_intensities": 0.0,
                    }
                ],
                remappings=[
                    ("/scan", LaunchConfiguration("topic_name")),
                ],
            ),
        ]
    )
