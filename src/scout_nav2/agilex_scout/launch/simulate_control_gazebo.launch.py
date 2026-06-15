# python imports
import os
from ament_index_python.packages import get_package_share_directory
from math import pi

# ros2 imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
	Command,
	FindExecutable,
	LaunchConfiguration,
	PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
	# Launch configuration variables specific to simulation

	odometry_source_arg = DeclareLaunchArgument(
		name="odometry_source",
		default_value="ground_truth",
		description="Odometry source (ground_truth or wheel encoders)",
		choices=["encoders", "ground_truth"],
	)

	rviz_arg = DeclareLaunchArgument(
		name="rviz",
		default_value="false",
		description="Open RViz with model display configuration",
		choices=["true", "false"],
	)

	lidar_type_arg = DeclareLaunchArgument(
		name="lidar_type",
		default_value="3d",
		description="choose lidar type: pointcloud with 3d lidar or laserscan with 2d lidar",
		choices=["3d", "2d"]
	)

	gui_arg = DeclareLaunchArgument(
		name="gui",
		default_value="true",
		description="Start Gazebo with GUI (set false for headless)",
		choices=["true", "false"],
	)

	# namespace 파라미터 추가 (기본값 빈 문자열 = 제안 시스템 기본 동작 유지)
	# 베이스라인에서만 namespace:=robot1 등으로 호출
	namespace_arg = DeclareLaunchArgument(
		name="namespace",
		default_value="",
		description="Robot namespace for baseline multi-robot (robot1/robot2/robot3). Empty = proposed system mode.",
	)

	bridge_clock_arg = DeclareLaunchArgument(
		name="bridge_clock",
		default_value="true",
		description="Bridge /clock from Gazebo to ROS2. Set false for robot2/3 to avoid multi-Gazebo /clock conflict on single domain.",
		choices=["true", "false"],
	)

	return LaunchDescription(
		[
			odometry_source_arg,
			rviz_arg,
			lidar_type_arg,
			gui_arg,
			namespace_arg,
			bridge_clock_arg,
			OpaqueFunction(function=launch_setup),
		]
	)


def launch_setup(context, *args, **kwargs):
	namespace = LaunchConfiguration("namespace").perform(context)
	use_namespace = namespace != ""
	bridge_clock = LaunchConfiguration("bridge_clock").perform(context) == "true"

	agilex_scout_dir = get_package_share_directory("agilex_scout")
	aws_small_warehouse_dir = get_package_share_directory("aws_robomaker_small_warehouse_world")

	# Gazebo 월드
	warehouse_world_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[aws_small_warehouse_dir, "/launch/no_roof_small_warehouse.launch.py"]
		),
		launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
	)

	# 브릿지 설정 파일 선택
	# - 제안 시스템 (namespace 없음): 기존 config (절대 경로, /clock 포함)
	# - 베이스라인 robot1 (bridge_clock:=true): /clock 포함 baseline config
	# - 베이스라인 robot2/3 (bridge_clock:=false): /clock 제외 baseline config ← /clock 충돌 방지
	if use_namespace:
		if bridge_clock:
			bridge_config = os.path.join(agilex_scout_dir, "config", "ros2_gz_bridge_baseline.yaml")
		else:
			bridge_config = os.path.join(agilex_scout_dir, "config", "ros2_gz_bridge_baseline_no_clock.yaml")
		qos_key = f"qos_overrides./{namespace}/tf_static.publisher.durability"
		use_sim_time = True   # Gazebo TF timestamp = sim time → use_sim_time:=True 필요
	else:
		bridge_config = os.path.join(agilex_scout_dir, "config", "ros2_gz_bridge_config.yaml")
		qos_key = "qos_overrides./tf_static.publisher.durability"
		use_sim_time = True

	bridge = Node(
		name="ros2_gz_bridge",
		package="ros_gz_bridge",
		executable="parameter_bridge",
		namespace=namespace if use_namespace else "",
		parameters=[{
			"config_file": bridge_config,
			qos_key: "transient_local",
		}],
		output="screen",
	)

	# URDF
	scout_description_file = os.path.join(agilex_scout_dir, "urdf", "robot.urdf.xacro")
	scout_description_content = Command(
		[
			FindExecutable(name="xacro"),
			" ",
			scout_description_file,
			" odometry_source:=", LaunchConfiguration("odometry_source"),
			" load_gazebo:=true",
			" simulation:=true",
			" lidar_type:=", LaunchConfiguration("lidar_type"),
		]
	)
	scout_description = {
		"robot_description": ParameterValue(scout_description_content, value_type=str)
	}

	# Robot State Publisher
	if use_namespace:
		# 베이스라인: namespace 아래 실행 → /robotN/tf, /robotN/tf_static
		# use_sim_time: True → RViz/Nav2와 동일한 시간 기준 (clock은 robot1 Gazebo에서 bridge)
		robot_state_publisher_node = Node(
			name="robot_state_publisher",
			package="robot_state_publisher",
			executable="robot_state_publisher",
			namespace=namespace,
			output="screen",
			parameters=[{"use_sim_time": True}, scout_description],
			remappings=[
				("/tf", "tf"),
				("/tf_static", "tf_static"),
			],
		)
		# spawn용 별도 publisher (TF 충돌 방지)
		robot_desc_publisher = Node(
			name="robot_state_publisher_for_spawn",
			package="robot_state_publisher",
			executable="robot_state_publisher",
			output="screen",
			parameters=[{"use_sim_time": True}, scout_description],
			remappings=[
				("/robot_description", "/scout/robot_description"),
				("/tf", f"/{namespace}_spawn_tf"),
				("/tf_static", f"/{namespace}_spawn_tf_static"),
			],
		)
	else:
		# 제안 시스템: 기존 방식 그대로
		robot_state_publisher_node = Node(
			name="robot_state_publisher",
			package="robot_state_publisher",
			executable="robot_state_publisher",
			output="screen",
			parameters=[{"use_sim_time": True}, scout_description],
			remappings=[
				("/joint_states", "/scout/joint_states"),
				("/robot_description", "/scout/robot_description"),
			],
		)
		robot_desc_publisher = None

	# Spawn 로봇
	spawn_robot_urdf_node = Node(
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

	# Static TF: map → odom
	if use_namespace:
		# 베이스라인: namespace 아래 → /robotN/tf_static
		# world→map
		static_tf = Node(
			package="tf2_ros",
			executable="static_transform_publisher",
			namespace=namespace,
			arguments=[
				"--x", "0.0", "--y", "0.0", "--z", "0.0",
				"--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
				"--frame-id", "world",
				"--child-frame-id", "map",
			],
			parameters=[{"use_sim_time": True}],
			remappings=[("/tf_static", "tf_static")],
		)
		# map→odom (rl3 역할: SLAM toolbox 초기화 전 Nav2가 필요로 하는 초기 변환)
		static_tf_map_odom = Node(
			package="tf2_ros",
			executable="static_transform_publisher",
			namespace=namespace,
			arguments=[
				"--x", "0.0", "--y", "0.0", "--z", "0.0",
				"--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
				"--frame-id", "map",
				"--child-frame-id", "odom",
			],
			parameters=[{"use_sim_time": True}],
			remappings=[("/tf_static", "tf_static")],
		)
	else:
		# 제안 시스템: 기존 방식
		static_tf = Node(
			package="tf2_ros",
			executable="static_transform_publisher",
			arguments=[
				"--x", "0.0", "--y", "0.0", "--z", "0.0",
				"--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
				"--frame-id", "world",
				"--child-frame-id", "map",
			],
			parameters=[{"use_sim_time": True}],
		)

	# RViz
	rviz2_file = os.path.join(agilex_scout_dir, "rviz", "model_display.rviz")
	# baseline 모드: /tf → /robotN/tf, /tf_static → /robotN/tf_static 로 remapping
	# (RViz는 기본적으로 /tf, /tf_static 구독 → 명시적으로 실제 토픽으로 연결)
	rviz2_remappings = [
		("/tf", f"/{namespace}/tf"),
		("/tf_static", f"/{namespace}/tf_static"),
	] if use_namespace else []
	rviz2_node = Node(
		package="rviz2",
		executable="rviz2",
		arguments=["-d", rviz2_file],
		parameters=[{"use_sim_time": use_sim_time}, scout_description],
		remappings=rviz2_remappings,
		condition=IfCondition(LaunchConfiguration("rviz")),
	)

	# Pointcloud to laserscan
	pointcloud_to_laserscan_node = Node(
		package='pointcloud_to_laserscan',
		executable='pointcloud_to_laserscan_node',
		name='pointcloud_to_laserscan_node',
		namespace=namespace if use_namespace else "",
		remappings=[
			('cloud_in', "/points" if not use_namespace else "points"),
			('scan', "/laser_scan" if not use_namespace else "laser_scan"),
		],
		parameters=[{
			'transform_tolerance': 0.05,
			'min_height': 0.0,
			'max_height': 1.0,
			'angle_min': -pi,
			'angle_max': pi,
			'angle_increment': pi / 180.0 / 2.0,
			'scan_time': 1/10,
			'range_min': 0.1,
			'range_max': 100.0,
			'use_inf': True,
		}],
		condition=IfCondition(PythonExpression(["'", LaunchConfiguration("lidar_type"), "'", " == '3d'"]))
	)

	nodes = [
		warehouse_world_launch,
		robot_state_publisher_node,
		spawn_robot_urdf_node,
		bridge,
		static_tf,
		rviz2_node,
		pointcloud_to_laserscan_node,
	]
	if robot_desc_publisher:
		nodes.append(robot_desc_publisher)
	if use_namespace:
		nodes.append(static_tf_map_odom)

	return nodes
