#!/usr/bin/env python3
"""
단일 Gazebo 월드에 여러 Scout 로봇 생성
용도: 기존 방식(단일 도메인) 비교 실험용
- 모든 로봇이 같은 ROS_DOMAIN_ID 사용
- 네임스페이스로만 구분
- O(N²) discovery overhead 측정
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 로봇 대수 설정
    num_robots_arg = DeclareLaunchArgument(
        name='num_robots',
        default_value='3',
        description='Number of robots to spawn'
    )

    num_robots = LaunchConfiguration('num_robots')

    # Gazebo 월드 실행
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

    # 로봇 설정
    agilex_scout_dir = get_package_share_directory("agilex_scout")
    scout_description_file = os.path.join(
        agilex_scout_dir,
        "urdf",
        "robot.urdf.xacro"
    )

    # 로봇 spawn 위치 (x, y, z 좌표)
    robot_positions = [
        {'name': 'robot1', 'x': '0.0', 'y': '0.0', 'z': '0.2346'},
        {'name': 'robot2', 'x': '3.0', 'y': '0.0', 'z': '0.2346'},
        {'name': 'robot3', 'x': '6.0', 'y': '0.0', 'z': '0.2346'},
    ]

    robot_spawners = []

    for i, pos in enumerate(robot_positions):
        namespace = pos['name']

        # URDF 생성
        scout_description_content = Command(
            [
                FindExecutable(name="xacro"),
                " ",
                scout_description_file,
                " odometry_source:=ground_truth",
                " load_gazebo:=true",
                " simulation:=true",
                " lidar_type:=3d",
                " robot_namespace:=", namespace
            ]
        )

        # Robot State Publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{
                'use_sim_time': True,
                'robot_description': ParameterValue(scout_description_content, value_type=str)
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )

        # Spawn Robot
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', f'scout_{namespace}',
                '-topic', f'/{namespace}/robot_description',
                '-x', pos['x'],
                '-y', pos['y'],
                '-z', pos['z'],
            ],
            output='screen'
        )

        # Bridge (Gazebo <-> ROS2)
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            arguments=[
                f'/model/scout_{namespace}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                f'/model/scout_{namespace}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
            remappings=[
                (f'/model/scout_{namespace}/odometry', f'/{namespace}/odom'),
                (f'/model/scout_{namespace}/cmd_vel', f'/{namespace}/cmd_vel'),
            ],
            output='screen'
        )

        # Static TF: world -> map
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace,
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', f'{namespace}/map',
            ],
            parameters=[{'use_sim_time': True}]
        )

        # Group all nodes for this robot
        robot_group = GroupAction([
            PushRosNamespace(namespace),
            robot_state_publisher,
            spawn_robot,
            bridge,
            static_tf,
        ])

        robot_spawners.append(robot_group)

    return LaunchDescription([
        num_robots_arg,
        warehouse_world_launch,
        *robot_spawners,
    ])
