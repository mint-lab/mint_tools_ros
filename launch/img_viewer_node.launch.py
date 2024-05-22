import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    raw_img_topic = DeclareLaunchArgument('raw_img', default_value='/raw_img')
    comp_img_topic = DeclareLaunchArgument('comp_img', default_value='/comp_img')

    viewer_node = Node(
        package='mint_tools_ros2',
        executable='image_viewer',
        output='screen',
        parameters=[{'raw_img': LaunchConfiguration('raw_img')},
                    {'comp_img': LaunchConfiguration('comp_img')}])
    
    return LaunchDescription([raw_img_topic, comp_img_topic, viewer_node])