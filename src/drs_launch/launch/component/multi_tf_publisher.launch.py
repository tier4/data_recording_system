import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('individual_params'),
            'config/default/multi_tf_static.yaml'
        ),
        description='Path to YAML configuration file containing all transforms'
    )
    
    publish_camera_optical_link_arg = DeclareLaunchArgument(
        'publish_camera_optical_link',
        default_value='true',
        description='Whether to publish camera optical link transforms'
    )
    
    # Launch the multi_tf_publisher node
    multi_tf_publisher_node = Node(
        package='multi_tf_publisher',
        executable='multi_tf_publisher',
        name='multi_tf_publisher',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'publish_camera_optical_link': LaunchConfiguration('publish_camera_optical_link')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        publish_camera_optical_link_arg,
        multi_tf_publisher_node
    ])