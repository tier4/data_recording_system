"""Launch the vehicle_can_node with a vehicle-specific YAML config."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        description="Absolute path to the vehicle YAML config file.",
    )

    node = Node(
        package="vehicle_can_decoder",
        executable="vehicle_can_node",
        name="vehicle_can_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_arg, node])
