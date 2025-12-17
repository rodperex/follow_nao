from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    
    # Get the path to the behavior tree XML file
    config_dir = os.path.join(
        get_package_share_directory('follow_nao'),
        'config'
    )
    bt_xml_file = os.path.join(config_dir, 'follow_behavior.xml')
    
    return LaunchDescription([
        
        # Launch entity tracker to publish target TF
        Node(
            package='follow_nao',
            executable='entity_tracker_fake_3d',
            name='entity_tracker_node',
            output='screen',
            parameters=[{
                'target_class': 'person',
                'source_frame': 'base_link',
                'target_frame': 'target',
                'optical_frame': 'CameraTop_optical_frame'
            }],
            remappings=[
                ('/input_detection_2d', '/detections_2d'),
                ('/camera_info', '/camera_rgb_info')
            ]
        ),
        
        # Launch YOLO to standard converter
        Node(
            package='follow_nao',
            executable='yolo_to_standard',
            name='yolo_to_standard_node',
            output='screen',
            remappings=[
                ('input_detection_3d', '/yolo/detections_3d'),
                ('input_detection_2d', '/yolo/detections'),
                ('output_detection_3d', '/detections_3d'),
                ('output_detection_2d', '/detections_2d')
            ]
        ),
        
        # Launch BehaviorTree follow node
        Node(
            package='follow_nao',
            executable='bt_follow_node',
            name='bt_follow_node',
            output='screen',
            parameters=[{
                'bt_xml': bt_xml_file,
                'tick_rate': 10.0
            }]
        ),
        
        # Launch YOLO detector
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('yolo_bringup'),
                    'launch',
                    'yolo.launch.py'
                )
            ),
            launch_arguments={
                'input_image_topic': '/image_rgb',
                'input_depth_topic': '/image_depth',
                'input_depth_info_topic': '/camera_rgb_info',
                'target_frame': 'CameraTop_frame',
                'depth_image_reliability': '1'
            }.items()
        ),
    ])
