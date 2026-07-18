#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Generate camera launch components with proper delay."""
    # Get launch configurations from context
    camera_id = LaunchConfiguration('camera_id').perform(context)
    param_root_dir = LaunchConfiguration('param_root_dir').perform(context)
    live_sensor = LaunchConfiguration('live_sensor').perform(context)
    set_readout_delay = LaunchConfiguration('set_readout_delay').perform(context)
    startup_delay = float(LaunchConfiguration('startup_delay').perform(context))
    use_v4l2_buffer_timestamps = LaunchConfiguration('use_v4l2_buffer_timestamps').perform(context)

    # Create the root container name
    root_container_name = f'camera{camera_id}_container'

    # Create component container node
    component_container = ComposableNodeContainer(
        name=root_container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(LaunchConfiguration('live_sensor')),
        output='screen'
    )

    # Load v4l2_camera composable node
    v4l2_camera_node = ComposableNode(
        package='v4l2_camera',
        plugin='v4l2_camera::V4L2Camera',
        name='v4l2_camera',
        namespace='',
        remappings=[
            ('image_raw', 'image_raw'),
            ('image_raw/compressed', 'image_raw/compressed'),
            ('image_raw/compressedDepth', 'image_raw/compressedDepth'),
            ('image_raw/theora', 'image_raw/theora')
        ],
        parameters=[
            f'{param_root_dir}/camera{camera_id}/v4l2_camera.param.yaml',
            {
                'camera_frame_id': f'camera{camera_id}/camera_optical_link',
                'camera_info_url': f'file://{param_root_dir}/camera{camera_id}/camera_info.yaml',
                'use_sensor_data_qos': False,
                'publish_rate': -1.0,
                'use_image_transport': False,
                'use_v4l2_buffer_timestamps': use_v4l2_buffer_timestamps.lower() == 'true'
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Load v4l2_camera into the container
    load_v4l2_camera = LoadComposableNodes(
        composable_node_descriptions=[v4l2_camera_node],
        target_container=component_container,
        condition=IfCondition(LaunchConfiguration('live_sensor'))
    )

    # Load accelerated_image_processor composable node
    accelerated_image_processor_node = ComposableNode(
        package='accelerated_image_processor',
        plugin='gpu_imgproc::GpuImgProc',
        name='accelerated_img_proc',
        namespace='',
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info'),
            ('image_raw/compressed', 'image_raw/compressed')
        ],
        parameters=[
            f'{param_root_dir}/camera{camera_id}/accelerated_image_processor.param.yaml'
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Load accelerated_image_processor into the container
    load_accelerated_image_processor = LoadComposableNodes(
        composable_node_descriptions=[accelerated_image_processor_node],
        target_container=component_container,
        condition=IfCondition(LaunchConfiguration('live_sensor'))
    )

    # Create readout setter node
    readout_setter_node = Node(
        package='c2_readout_delay_setter',
        executable='c2_readout_delay_setter',
        name=f'readout_setter_{camera_id}',
        namespace='',
        parameters=[
            f'{param_root_dir}/camera{camera_id}/readout_delay.param.yaml',
            {'target_v4l2_node': 'v4l2_camera'}
        ],
        condition=IfCondition(
            'true' if live_sensor.lower() == 'true' and set_readout_delay.lower() == 'true' else 'false'
        ),
        output='screen'
    )

    # Group all actions with explicit namespace
    camera_group = GroupAction([
        PushRosNamespace(f'/sensing/camera/camera{camera_id}'),
        component_container,
        load_v4l2_camera,
        load_accelerated_image_processor,
        readout_setter_node
    ])

    # If delay is specified, wrap in TimerAction
    if startup_delay > 0:
        return [TimerAction(
            period=startup_delay,
            actions=[camera_group]
        )]
    else:
        return [camera_group]


def generate_launch_description():
    # Declare launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        description='Camera ID number'
    )

    param_root_dir_arg = DeclareLaunchArgument(
        'param_root_dir',
        description='Root directory for parameter files'
    )

    live_sensor_arg = DeclareLaunchArgument(
        'live_sensor',
        description='Whether to boot sensor drivers for online mode'
    )

    set_readout_delay_arg = DeclareLaunchArgument(
        'set_readout_delay',
        default_value='true',
        description='Whether to set readout delay'
    )

    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay',
        default_value='0.0',
        description='Delay in seconds before starting the camera'
    )

    use_v4l2_buffer_timestamps_arg = DeclareLaunchArgument(
        'use_v4l2_buffer_timestamps',
        default_value='true',
        description='Use v4l2 buffer timestamps instead of ROS system time'
    )

    # Return the launch description
    return LaunchDescription([
        camera_id_arg,
        param_root_dir_arg,
        live_sensor_arg,
        set_readout_delay_arg,
        startup_delay_arg,
        use_v4l2_buffer_timestamps_arg,
        OpaqueFunction(function=launch_setup)
    ])
