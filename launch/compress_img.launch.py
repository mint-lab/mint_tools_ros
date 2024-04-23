import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    img_topic = DeclareLaunchArgument('origin_img', default_value='/rgb_img')
    compress_node = Node(
        package='mint_tools_ros2',
        executable='compress_img',
        output='screen',
        parameters=[{'origin_img': LaunchConfiguration('origin_img')}])
    
    return LaunchDescription([img_topic, compress_node])