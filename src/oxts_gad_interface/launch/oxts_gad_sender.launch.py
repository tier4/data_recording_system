from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generates the launch description for the OxTS GAD Sender Node.

    This node subscribes to VelocityReport and sends GAD packets to OxTS device via UDP.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'velocity_report_topic',
            default_value='/vehicle/velocity_report',
            description='Input VelocityReport topic.',
        ),
        DeclareLaunchArgument(
            'ins_ip',
            default_value='192.168.4.250',
            description='IP address of the OxTS GNSS/INS unit.',
        ),
        DeclareLaunchArgument(
            'stream_id',
            default_value='157',
            description='GAD Stream ID for the OxTS unit (must be 128-255).',
        ),
        DeclareLaunchArgument(
            'gad_speed_std_dev',
            default_value='0.1',
            description='Assumed standard deviation of the speed measurement in m/s.',
        ),
        DeclareLaunchArgument(
            'gad_latency',
            default_value='0.02',
            description='Estimated latency of the velocity data in seconds.',
        ),
        DeclareLaunchArgument(
            'aiding_lever_arm',
            default_value='[-0.6895, 0.0, -1.9705]',
            description='[X, Y, Z] offset in meters from the INS to the measurement point.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='10.0',
            description='Rate (Hz) at which GAD packets are sent.',
        ),
        DeclareLaunchArgument(
            'log-level',
            default_value=TextSubstitution(text='INFO'),
            description='Logging level (DEBUG, INFO, WARN, ERROR, FATAL)',
        ),
        Node(
            package='oxts_gad_interface',
            executable='oxts_gad_sender_node',
            name='oxts_gad_sender_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'velocity_report_topic': LaunchConfiguration('velocity_report_topic'),
                    'ins_ip': LaunchConfiguration('ins_ip'),
                    'stream_id': LaunchConfiguration('stream_id'),
                    'gad_speed_std_dev': LaunchConfiguration('gad_speed_std_dev'),
                    'gad_latency': LaunchConfiguration('gad_latency'),
                    'aiding_lever_arm': LaunchConfiguration('aiding_lever_arm'),
                    'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log-level')],
        ),
    ])
