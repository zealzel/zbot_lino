from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensor_ip_arg = DeclareLaunchArgument("sensor_ip")
    host_ip_arg = DeclareLaunchArgument("host_ip")
    remap_topic_arg = DeclareLaunchArgument("remap_topic")
    return LaunchDescription([
        sensor_ip_arg,
        host_ip_arg,
        remap_topic_arg,
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_lifecycle_node",
            name="sick_safetyscanners2_lifecycle_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "scan",
                 "sensor_ip": LaunchConfiguration("sensor_ip"),
                 "host_ip": LaunchConfiguration("host_ip"),
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 0,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            remappings=[
                ("/scan", LaunchConfiguration("remap_topic_arg")),
            ],
        ),
    ])
