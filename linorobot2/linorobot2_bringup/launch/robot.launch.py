import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# In linorobot2_hardware/firmware/src/firmware.ino
# Change the following line
# RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
# into
# RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "/lino2", &support));


def get_path(package_name, subpaths):
    return PathJoinSubstitution([FindPackageShare(package_name)] + subpaths)


def generate_launch_description():
    # namespace = "/lino2"

    # real robot: zbotlino(use rplidar)|zbotlinosick1|zbotlinosick2
    robot_base = os.getenv("LINOROBOT2_BASE", "zbotlinosick2")
    if robot_base in ["zbotlinosick2", "zbotlino2"]:
        sensor_launch = "sensors_sick"
    else:
        sensor_launch = "sensors"

    sensors_launch_path = get_path(
        "linorobot2_bringup", ["launch", f"{sensor_launch}.launch.py"]
    )
    description_launch_path = get_path(
        "linorobot2_description", ["launch", "description.launch.py"]
    )
    ekf_config_path = get_path("linorobot2_base", ["config", "ekf.yaml"])

    namespace_arg = DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="namespace",
    )
    base_serial_port_arg = DeclareLaunchArgument(
        name="base_serial_port",
        default_value="/dev/ttyACM0",
        description="Linorobot Base Serial Port",
    )
    joy_arg = DeclareLaunchArgument(
        name="joy", default_value="false", description="Use Joystick"
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path],
        namespace=LaunchConfiguration("namespace"),
        remappings=[
            ("odometry/filtered", "odom"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_path),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
        }.items(),
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
        }.items(),
    )
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        namespace=LaunchConfiguration("namespace"),
        arguments=["serial", "--dev", LaunchConfiguration("base_serial_port")],
    )
    return LaunchDescription(
        [
            base_serial_port_arg,
            joy_arg,
            namespace_arg,
            description_launch,
            sensor_launch,
            ekf,
            micro_ros_agent,
        ]
    )
