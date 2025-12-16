from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
   
    return LaunchDescription([
  
        Node(
            package='follow_nao',
            executable='motion_control_2d',
            name='motion_control_2d_node',
            output='screen',
            parameters=[{
            'avoidance_distance': 0.5,
            'base_frame': 'base_link',
            'max_linear_speed': 1.0,
            'max_angular_speed': 1.0,
            'optical_frame': 'CameraTop_optical_frame',
            'target_class': 'person',
            }],
            remappings=[
            ('/sonar', '/sensors/sonar'),
            ('/vel', '/target'),
            ('/touch', '/sensors/touch'),
            ('/camera_info', '/camera_rgb_info'),
            ('/input_detection_2d', '/detections_2d'),
            ]
        ),
    ])