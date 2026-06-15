#!/usr/bin/env python3
"""
단일 로봇 spawn + RViz (센서 시각화)
기존 rl1 기능을 멀티 로봇으로 확장

사용법:
  export ROS_DOMAIN_ID=1
  ros2 launch agilex_scout spawn_robot_with_rviz.launch.py robot_name:=robot1 x:=0.0 y:=0.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
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

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for sensor visualization',
        choices=['true', 'false']
    )

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

    # Gazebo-ROS Bridge config (동적 생성)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_name,
        arguments=[
            # cmd_vel
            ['/model/scout_', robot_name, '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            # odometry
            ['/model/scout_', robot_name, '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            # laser scan
            ['/model/scout_', robot_name, '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            # pointcloud
            ['/model/scout_', robot_name, '/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
            # imu
            ['/model/scout_', robot_name, '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        ],
        remappings=[
            (['/model/scout_', robot_name, '/cmd_vel'], 'cmd_vel'),
            (['/model/scout_', robot_name, '/odometry'], 'odom'),
            (['/model/scout_', robot_name, '/scan'], 'scan'),
            (['/model/scout_', robot_name, '/points'], 'points'),
            (['/model/scout_', robot_name, '/imu'], 'imu'),
        ],
        output='screen'
    )

    # Static TF: map -> odom
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

    # RViz for sensor visualization
    rviz_config_file = os.path.join(
        agilex_scout_dir,
        "rviz",
        "model_display.rviz"
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot_name,
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        rviz_arg,
        robot_state_publisher,
        spawn_robot,
        bridge,
        static_tf_map_odom,
        rviz2_node,
    ])
