#!/usr/bin/env python3
"""
단일 Gazebo 물리 엔진만 실행
용도: 모든 로봇이 같은 물리 공간을 공유
ROS_DOMAIN_ID와 무관하게 실행
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # Gazebo 월드 (AWS Warehouse)
    aws_small_warehouse_dir = get_package_share_directory(
        "aws_robomaker_small_warehouse_world"
    )
    warehouse_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                aws_small_warehouse_dir,
                "/launch/no_roof_small_warehouse.launch.py",
            ]
        )
    )

    # Static TF: world -> map (공통)
    static_tf_world_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "world",
            "--child-frame-id", "map",
        ],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        warehouse_world_launch,
        static_tf_world_map,
    ])
