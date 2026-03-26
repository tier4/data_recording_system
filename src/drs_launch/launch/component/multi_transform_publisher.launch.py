import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicle_id',
        default_value=os.environ.get('VEHICLE_ID', 'default'),
        description='Vehicle ID used to select the individual_params config directory'
    )

    publish_camera_optical_link_arg = DeclareLaunchArgument(
        'publish_camera_optical_link',
        default_value='true',
        description='Whether to publish camera optical link transforms'
    )

    periodic_publish_arg = DeclareLaunchArgument(
        'periodic_publish',
        default_value='true',
        description='Whether to publish transforms periodically instead of as static transforms'
    )

    publish_period_arg = DeclareLaunchArgument(
        'publish_period',
        default_value='1.0',
        description='Period in seconds for periodic publishing (only used when periodic_publish is true)'
    )

    # Launch the multi_transform_publisher node
    multi_transform_publisher_node = Node(
        package='multi_transform_publisher',
        executable='multi_transform_publisher',
        name='multi_transform_publisher',
        parameters=[{
            'config_file': PathJoinSubstitution([
                get_package_share_directory('individual_params'),
                'config',
                LaunchConfiguration('vehicle_id'),
                'multi_tf_static.yaml',
            ]),
            'publish_camera_optical_link': LaunchConfiguration('publish_camera_optical_link'),
            'periodic_publish': LaunchConfiguration('periodic_publish'),
            'publish_period': LaunchConfiguration('publish_period')
        }],
        output='screen'
    )

    return LaunchDescription([
        vehicle_id_arg,
        publish_camera_optical_link_arg,
        periodic_publish_arg,
        publish_period_arg,
        multi_transform_publisher_node
    ])
