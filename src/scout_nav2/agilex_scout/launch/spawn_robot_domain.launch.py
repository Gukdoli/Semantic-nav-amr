#!/usr/bin/env python3
"""
단일 로봇을 Gazebo에 spawn하고 ROS2 노드들 실행
실행 전 ROS_DOMAIN_ID를 설정해야 함

사용법:
  export ROS_DOMAIN_ID=1
  ros2 launch agilex_scout spawn_robot_domain.launch.py robot_name:=robot1 x:=0.0 y:=0.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Namespace for the robot (robot1, robot2, robot3, ...)'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.2346')

    robot_name = LaunchConfiguration('robot_name')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')

    # URDF/XACRO 파일
    agilex_scout_dir = get_package_share_directory("agilex_scout")
    scout_description_file = os.path.join(
        agilex_scout_dir,
        "urdf",
        "robot.urdf.xacro"
    )

    # XACRO 처리
    scout_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            scout_description_file,
            " odometry_source:=ground_truth",
            " load_gazebo:=true",
            " simulation:=true",
            " lidar_type:=3d",
            " robot_namespace:=", robot_name
        ]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(scout_description_content, value_type=str)
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ['scout_', robot_name],
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
            '-topic', [robot_name, '/robot_description'],
        ],
        output='screen'
    )

    # Gazebo-ROS Bridge
    # Gazebo 토픽을 현재 도메인의 ROS 토픽으로 변환
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/model/scout_', robot_name, '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            ['/model/scout_', robot_name, '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            ['/model/scout_', robot_name, '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        ],
        remappings=[
            (['/model/scout_', robot_name, '/odometry'], [robot_name, '/odom']),
            (['/model/scout_', robot_name, '/cmd_vel'], [robot_name, '/cmd_vel']),
            (['/model/scout_', robot_name, '/scan'], [robot_name, '/scan']),
        ],
        output='screen'
    )

    # Static TF: map -> robot_namespace/odom
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', [robot_name, '/odom'],
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        robot_state_publisher,
        spawn_robot,
        bridge,
        static_tf_map_odom,
    ])
