from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/arduino",
        description="Serial device path for Arduino OTOS bridge",
    )

    params_file = os.path.join(
        get_package_share_directory("otos_serial_driver"),
        "config",
        "otos_params.yaml",
    )

    node = Node(
        package="otos_serial_driver",
        executable="otos_serial_node",
        name="otos_serial_node",
        output="screen",
        parameters=[params_file, {"serial_port": LaunchConfiguration("serial_port")}],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="otos_static_tf",
        arguments=[
            "--x", "0.1017",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "otos_link",
            ],
        )

    return LaunchDescription([serial_port_arg, node, static_tf])
