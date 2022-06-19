# Copyright 2022 Siddharth Saha


from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

import os


def generate_launch_description():

    bringup_dir = get_package_share_directory('car_heading_generator')

    param_file_path = os.path.join(
        bringup_dir,
        'param',
        'car_heading_generator.param.yaml'
    )

    return LaunchDescription(
        [
            Node(
                package="car_heading_generator",
                executable="car_heading_generator_node_exe",
                name="car_heading_generator_node",
                output="screen",
                parameters=[param_file_path],
                remappings=[
                    ("gps_front", "/gps_front/gps"),
                    ("gps_back", "/gps_back/gps")
                ],
            ),
        ]
    )
