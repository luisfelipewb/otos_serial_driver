from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
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

    return LaunchDescription([serial_port_arg, node])
