from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the GAD Converter Node.

    This launch file starts the gad_converter_node and allows setting
    all of its parameters from the command line or by modifying the default values here.
    """
    return LaunchDescription([
        # --- Declare all launch arguments for the node parameters ---

        DeclareLaunchArgument(
            'can_topic',
            default_value='/vehicle/from_can_bus',
            description='The ROS2 topic for incoming CAN messages.'
        ),
        DeclareLaunchArgument(
            'ins_ip',
            default_value='192.168.4.250',
            description='The IP address of the OxTS GNSS/INS unit.'
        ),
        DeclareLaunchArgument(
            'stream_id',
            default_value='157',
            description='GAD Stream ID for the OxTS unit (must be 128-255).'
        ),
        DeclareLaunchArgument(
            'gad_speed_std_dev',
            default_value='0.1',
            description='Assumed standard deviation of the speed measurement in m/s.'
        ),
        DeclareLaunchArgument(
            'gad_latency',
            default_value='0.02',
            description='Estimated latency of the CAN speed data in seconds.'
        ),
        DeclareLaunchArgument(
            'dbc_file_path',
            default_value='/opt/drs/config/vehicle.dbc', 
            description='Absolute path to the DBC file for CAN message decoding.'
        ),
        DeclareLaunchArgument(
            'can_message_name',
            default_value='VEHICLE_SPEED_RPT',
            description='The name of the CAN message containing the speed signal.'
        ),
        DeclareLaunchArgument(
            'can_signal_name',
            default_value='VEHICLE_SPEED',
            description='The name of the CAN signal for vehicle speed.'
        ),
        DeclareLaunchArgument(
            'aiding_lever_arm',
            default_value='[-0.6895, 0.0, -1.9705]',
            description='[X, Y, Z] offset in meters from the INS to the measurement point.'
        ),
        DeclareLaunchArgument(
            'log-level',
            default_value=TextSubstitution(text=str('INFO')),
            description='Logging level (e.g., DEBUG, INFO, WARN, ERROR, FATAL)'
        ),

        # --- Node Definition ---
        Node(
            package='oxts_gad_interface',
            executable='gad_converter_node',
            name='gad_converter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    # Use LaunchConfiguration to pass the arguments to the node
                    'can_topic': LaunchConfiguration('can_topic'),
                    'ins_ip': LaunchConfiguration('ins_ip'),
                    'stream_id': LaunchConfiguration('stream_id'),
                    'gad_speed_std_dev': LaunchConfiguration('gad_speed_std_dev'),
                    'gad_latency': LaunchConfiguration('gad_latency'),
                    'dbc_file_path': LaunchConfiguration('dbc_file_path'),
                    'can_message_name': LaunchConfiguration('can_message_name'),
                    'can_signal_name': LaunchConfiguration('can_signal_name'),
                    'aiding_lever_arm': LaunchConfiguration('aiding_lever_arm'),
                }
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log-level')]
        )
    ])
