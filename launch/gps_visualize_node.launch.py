from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mapviz",
                executable="mapviz",
                name="mapviz",
                output="screen",
            ),
            Node(
                package="swri_transform_util",
                executable="initialize_origin.py",
                name="initialize_origin",
                parameters=[
                    {"name": "local_xy_frame", "value": "map"},
                    {"name": "local_xy_origin", "value": "hywc"},
                    {
                        "name": "local_xy_origins",
                        "value": """[
                    {"name": "swri",
                        "latitude": 29.45196669,
                        "longitude": -98.61370577,
                        "altitude": 233.719,
                        "heading": 0.0},
                    {"name": "hywc",
                        "latitude": 37.558767,
                        "longitude": 127.048620,
                        "altitude": 62.627,
                        "heading": 0.0},
                    {"name": "goch",
                        "latitude": 37.498137,
                        "longitude": 126.867061,
                        "altitude": 62.627,
                        "heading": 0.0}

                ]""",
                    },
                ],
                remappings=[("fix", "/ublox/fix")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="swri_transform",
                arguments=["0", "0", "0", "0", "0", "0", "map", "origin"],
            ),
        ]
    )
