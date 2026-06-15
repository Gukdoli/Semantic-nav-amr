#!/usr/bin/env python3
"""Baseline multi-robot Gazebo launch.

All robots stay in ROS_DOMAIN_ID=0 and are separated only by namespace.
Each Gazebo instance is isolated with IGN_PARTITION.
Only robot1 should bridge the global /clock topic.
"""

import os
from math import pi
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    lidar_type = LaunchConfiguration("lidar_type").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    bridge_clock = LaunchConfiguration("bridge_clock").perform(context) == "true"
    use_sim_time = True

    # Gazebo 월드 실행
    aws_small_warehouse_dir = get_package_share_directory("aws_robomaker_small_warehouse_world")
    warehouse_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [aws_small_warehouse_dir, "/launch/no_roof_small_warehouse.launch.py"]
        ),
        launch_arguments={"gui": gui}.items(),
    )

    agilex_scout_dir = get_package_share_directory("agilex_scout")
    bridge_config_name = (
        "ros2_gz_bridge_baseline.yaml"
        if bridge_clock
        else "ros2_gz_bridge_baseline_no_clock.yaml"
    )
    ros2_gz_bridge_file = os.path.join(
        agilex_scout_dir,
        "config",
        bridge_config_name,
    )

    bridge = Node(
        name="ros2_gz_bridge",
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        parameters=[{"config_file": ros2_gz_bridge_file}],
        output="screen",
    )

    # URDF
    scout_description_file = os.path.join(agilex_scout_dir, "urdf", "robot.urdf.xacro")
    scout_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        scout_description_file,
        " odometry_source:=ground_truth",
        " load_gazebo:=true",
        " simulation:=true",
        f" lidar_type:={lidar_type}",
    ])
    scout_description = {
        "robot_description": ParameterValue(scout_description_content, value_type=str)
    }

    # Robot state publisher keeps the robot-local TF tree in /robotN/tf and /robotN/tf_static.
    robot_state_publisher = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, scout_description],
        remappings=[
            ("/clock", "clock"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # A separate publisher exposes /scout/robot_description for spawning without polluting TF.
    robot_desc_publisher = Node(
        name="robot_state_publisher_for_spawn",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}, scout_description],
        remappings=[
            ("/joint_states", f"/{namespace}_spawn_joint_states"),
            ("/robot_description", "/scout/robot_description"),
            ("/tf", f"/{namespace}_spawn_tf"),
            ("/tf_static", f"/{namespace}_spawn_tf_static"),
        ],
    )

    # Gazebo에 로봇 spawn (IGN_PARTITION으로 인스턴스 분리)
    spawn_robot = Node(
        name="spawn_robot_urdf",
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "scout_v2",
            "-topic", "/scout/robot_description",
            "-x", "0", "-y", "0", "-z", "0.2346",
            "-R", "0", "-P", "0", "-Y", "0",
        ],
        output="screen",
    )

    # map->odom must have a single authority. slam_toolbox publishes it in the
    # namespaced Nav2 stack, so we intentionally do not add a static publisher here.
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        namespace=namespace,
        name="pointcloud_to_laserscan_node",
        remappings=[
            ("/clock", "clock"),
            ("cloud_in", "points"),
            ("scan", "laser_scan_localization"),
        ],
        parameters=[{
            "use_sim_time": use_sim_time,
            "transform_tolerance": 0.05,
            "min_height": 0.0,
            "max_height": 1.0,
            "angle_min": -pi,
            "angle_max": pi,
            "angle_increment": pi / 180.0 / 2.0,
            "scan_time": 0.1,
            "range_min": 0.1,
            "range_max": 100.0,
            "use_inf": True,
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("lidar_type"), "'", " == '3d'"])
        ),
    )

    # Optional robot-local RViz for debugging.
    rviz2_file = os.path.join(
        agilex_scout_dir,
        "rviz",
        "model_display.rviz",
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_file],
        parameters=[{"use_sim_time": use_sim_time}, scout_description],
        remappings=[
            ("/clock", f"/{namespace}/clock"),
            ("/tf", f"/{namespace}/tf"),
            ("/tf_static", f"/{namespace}/tf_static"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    nodes = [
        warehouse_world_launch,
        robot_desc_publisher,
        robot_state_publisher,
        spawn_robot,
        bridge,
        pointcloud_to_laserscan,
        rviz2_node,
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot1",
            description="Robot namespace (robot1, robot2, robot3)",
        ),
        DeclareLaunchArgument(
            "bridge_clock",
            default_value="true",
            description="Bridge the global /clock topic from this Gazebo instance.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Gazebo with GUI",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "lidar_type",
            default_value="3d",
            description="Lidar type: 3d or 2d",
            choices=["3d", "2d"],
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Launch RViz",
            choices=["true", "false"],
        ),
        OpaqueFunction(function=launch_setup),
    ])
