from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='default',
            description='Vehicle identifier for DBC/config selection.',
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('vehicle_info_converter'),
                'config',
                'default.param.yaml',
            ]),
            description='Path to the parameter file.',
        ),
        DeclareLaunchArgument(
            'can_topic',
            default_value='/vehicle/from_can_bus',
            description='Input CAN topic.',
        ),
        DeclareLaunchArgument(
            'velocity_report_topic',
            default_value='/vehicle/velocity_report',
            description='Output VelocityReport topic.',
        ),
        DeclareLaunchArgument(
            'log-level',
            default_value='INFO',
            description='Logging level (DEBUG, INFO, WARN, ERROR, FATAL)',
        ),
        DeclareLaunchArgument(
            'param_root_dir',
            default_value='/opt/drs/config',
            description='Root directory for parameters and DBC files.',
        ),
        Node(
            package='vehicle_info_converter',
            executable='vehicle_info_converter_node',
            name='vehicle_info_converter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('param_file'),
                {
                    'vehicle_id': LaunchConfiguration('vehicle_id'),
                    'can_topic': LaunchConfiguration('can_topic'),
                    'velocity_report_topic': LaunchConfiguration('velocity_report_topic'),
                    'param_root_dir': LaunchConfiguration('param_root_dir'),
                },
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log-level')],
        ),
    ])
