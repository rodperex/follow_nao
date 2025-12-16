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
            executable='motion_control_3d',
            name='motion_control_3d_node',
            output='screen',
            parameters=[{
            'min_distance': 1.0,
            'avoidance_distance': 0.5,
            'base_frame': 'base_link'
            }],
            remappings=[
            ('/sonar', '/sensors/sonar'),
            ('/vel', '/target')
            ]
        ),
    ])