import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    img_topic = DeclareLaunchArgument('comp_img', default_value='/comp_img')
    compress_node = Node(
        package='mint_tools_ros2',
        executable='decompress_img',
        output='screen',
        parameters=[{'comp_img': LaunchConfiguration('comp_img')}])
    
    return LaunchDescription([img_topic, compress_node])