#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


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
        default_value='True',
        description='Whether to set readout delay'
    )
    
    # Get launch configurations
    camera_id = LaunchConfiguration('camera_id')
    param_root_dir = LaunchConfiguration('param_root_dir')
    live_sensor = LaunchConfiguration('live_sensor')
    set_readout_delay = LaunchConfiguration('set_readout_delay')
    
    # Create the root container name
    root_container_name = ['camera', camera_id, '_container']
    
    # Create component container node
    component_container = ComposableNodeContainer(
        name=root_container_name,
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(live_sensor)
    )
    
    # Load v4l2_camera composable node
    v4l2_camera_node = ComposableNode(
        package='v4l2_camera',
        plugin='v4l2_camera::V4L2Camera',
        name='v4l2_camera',
        namespace=['camera', camera_id],
        remappings=[
            ('image_raw', 'image_raw'),
            ('image_raw/compressed', 'image_raw/compressed'),
            ('image_raw/compressedDepth', 'image_raw/compressedDepth'),
            ('image_raw/theora', 'image_raw/theora')
        ],
        parameters=[
            [param_root_dir, '/', 'camera', camera_id, '/v4l2_camera.param.yaml'],
            {
                'camera_frame_id': ['camera', camera_id, '/camera_optical_link'],
                'camera_info_url': ['file://', param_root_dir, '/', 'camera', camera_id, '/camera_info.yaml'],
                'use_sensor_data_qos': True,
                'publish_rate': -1.0,
                'use_image_transport': False
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # Load v4l2_camera into the container
    load_v4l2_camera = LoadComposableNodes(
        composable_node_descriptions=[v4l2_camera_node],
        target_container=component_container,
        condition=IfCondition(live_sensor)
    )
    
    # Load accelerated_image_processor composable node
    accelerated_image_processor_node = ComposableNode(
        package='accelerated_image_processor',
        plugin='gpu_imgproc::GpuImgProc',
        name='accelerated_img_proc',
        namespace=['camera', camera_id],
        remappings=[
            ('image_raw', 'image_raw'),
            ('camera_info', 'camera_info'),
            ('image_raw/compressed', 'image_raw/compressed')
        ],
        parameters=[
            [param_root_dir, '/', 'camera', camera_id, '/accelerated_image_processor.param.yaml']
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # Load accelerated_image_processor into the container
    load_accelerated_image_processor = LoadComposableNodes(
        composable_node_descriptions=[accelerated_image_processor_node],
        target_container=component_container,
        condition=IfCondition(live_sensor)
    )
    
    # Create readout setter node
    readout_setter_node = Node(
        package='c2_readout_delay_setter',
        executable='c2_readout_delay_setter',
        name=['readout_setter_', camera_id],
        namespace=['camera', camera_id],
        parameters=[
            [param_root_dir, '/', 'camera', camera_id, '/readout_delay.param.yaml'],
            {'target_v4l2_node': 'v4l2_camera'}
        ],
        condition=IfCondition(Command([
            'bash -c "',
            'if [ "', live_sensor, '" = "True" ] && [ "', set_readout_delay, '" = "True" ]; then ',
            'echo true; else echo false; fi"'
        ]))
    )
    
    # Group all actions
    camera_group = GroupAction([
        component_container,
        load_v4l2_camera,
        load_accelerated_image_processor,
        readout_setter_node
    ])
    
    # Return the launch description
    return LaunchDescription([
        camera_id_arg,
        param_root_dir_arg,
        live_sensor_arg,
        set_readout_delay_arg,
        camera_group
    ])