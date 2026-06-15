"""Unified bringup for the semantic navigation AMR (M1).

Brings up Gazebo + Scout v2 + Nav2 + RViz with one launch. The existing
agilex_scout / scout_nav2 launches are reused via IncludeLaunchDescription and
are NOT modified. Nav2 is started after a short delay so Gazebo's /clock and TF
are up first.

Nav2 backend is selectable via the `nav2` arg:
  - bringup (default): standard `nav2_bringup` + the map under maps/test.yaml.
    This is what actually drives on the real machine (the old `rl4` recipe).
  - scout: scout_nav2/nav2.launch.py (slam_toolbox/amcl). NOTE its localization
    map path is hardcoded to another machine, so localization typically fails.
  - none: no Nav2 (just sim + RViz).

A static identity map->odom TF (`publish_map_odom_tf`, the rl3 workaround) is
published because odometry is ground-truth, so map==odom is correct and it
completes the TF tree even when no localizer publishes map->odom.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    agilex_scout_dir = get_package_share_directory("agilex_scout")
    scout_nav2_dir = get_package_share_directory("scout_nav2")
    bringup_dir = get_package_share_directory("semantic_nav_bringup")

    nav2_mode = LaunchConfiguration("nav2").perform(context)
    map_file = LaunchConfiguration("map").perform(context)
    nav2_delay = float(LaunchConfiguration("nav2_delay").perform(context))

    # Gazebo world + Scout v2 (starts immediately). rviz left at default (false).
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agilex_scout_dir, "launch", "simulate_control_gazebo.launch.py"
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "lidar_type": LaunchConfiguration("lidar_type"),
            "odometry_source": LaunchConfiguration("odometry_source"),
        }.items(),
    )

    # Static map -> odom identity TF (rl3 workaround; see module docstring).
    static_map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("publish_map_odom_tf")),
    )

    actions = [gazebo_launch, static_map_odom_tf]

    # --- Detection-target objects (M2): spawn into the running world ---
    # Spawned via `ros_gz_sim create` (same mechanism as the robot), so the reused
    # AWS warehouse world SDF is left untouched. Each model uses a Fuel mesh visual
    # with a PRIMITIVE collision (see models/<name>/model.sdf) to avoid the
    # mesh-collision assimp crash on this machine. Poses are open-floor spots in
    # front of the robot's start; tune as needed.
    if LaunchConfiguration("spawn_objects").perform(context) == "true":
        # (entity_name, model_dir, x, y, z, yaw). Poses are gz world coords (== map
        # frame here, since odom is ground-truth and map->odom is identity). yaw is
        # radians. entity_name must be unique per spawn; model_dir picks the SDF
        # under models/<model_dir>, so two instances can share one model.
        # Two fire extinguishers are placed >3 m apart (M3 instance-separation
        # check): merge_distance is 0.5 m, so they stay distinct in the map.
        objects = [
            ("fire_extinguisher", "fire_extinguisher",
             0.6699510216712952, 5.770341396331787, 0.0, -1.498456),
            # Second instance, 4 m south of the first (tune if it lands in a wall).
            ("fire_extinguisher_2", "fire_extinguisher",
             0.6699510216712952, 1.770341396331787, 0.0, -1.498456),
            ("chair", "chair",
             -3.3449838161468506, -4.413803577423096, 0.0, 0.070288),
        ]
        object_nodes = [
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{name}",
                arguments=[
                    "-name", name,
                    "-file", os.path.join(bringup_dir, "models", model, "model.sdf"),
                    "-x", str(ox), "-y", str(oy), "-z", str(oz), "-Y", str(oyaw),
                ],
                output="screen",
            )
            for name, model, ox, oy, oz, oyaw in objects
        ]
        # Gazebo must be up first; spawn after a short delay (before Nav2 starts).
        actions.append(TimerAction(period=5.0, actions=object_nodes))

    # --- Nav2 backend selection ---
    nav2_launch = None
    if nav2_mode == "scout":
        # scout_nav2 stack (also launches its own RViz with nav2.rviz)
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scout_nav2_dir, "launch", "nav2.launch.py")
            ),
            launch_arguments={
                "simulation": "true",
                "slam": LaunchConfiguration("slam"),
                "localization": LaunchConfiguration("localization"),
            }.items(),
        )
    elif nav2_mode == "bringup":
        # standard nav2_bringup + our map (the working 'rl4' recipe)
        nav2_bringup_dir = get_package_share_directory("nav2_bringup")
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "map": map_file,
            }.items(),
        )
    # nav2_mode == "none": leave nav2_launch as None

    if nav2_launch is not None:
        actions.append(TimerAction(period=nav2_delay, actions=[nav2_launch]))

    # --- Perception pipeline (M3): object_detector + semantic_map ---
    # Started after the sim is up so camera topics, camera_info and the TF tree
    # (map -> ... -> camera_color_optical_frame) are available before the nodes
    # try their first inference / TF lookup.
    if LaunchConfiguration("perception").perform(context) == "true":
        object_detector_params = os.path.join(
            bringup_dir, "params", "object_detector.yaml"
        )
        semantic_map_params = os.path.join(
            bringup_dir, "params", "semantic_map.yaml"
        )
        perception_nodes = [
            Node(
                package="object_detector",
                executable="object_detector_node",
                name="object_detector_node",
                parameters=[object_detector_params, {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="semantic_map",
                executable="semantic_map_node",
                name="semantic_map_node",
                parameters=[semantic_map_params, {"use_sim_time": True}],
                output="screen",
            ),
        ]
        actions.append(TimerAction(period=nav2_delay + 2.0, actions=perception_nodes))

    # RViz: scout mode brings its own; for bringup/none we launch one here.
    if nav2_mode != "scout":
        rviz_config = os.path.join(bringup_dir, "rviz", "semantic_nav.rviz")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory("semantic_nav_bringup")
    default_map = os.path.join(bringup_dir, "maps", "test.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value="true",
                description="Start Gazebo with GUI (set false for headless).",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                name="lidar_type",
                default_value="3d",
                description="Lidar type forwarded to the robot launch.",
                choices=["3d", "2d"],
            ),
            DeclareLaunchArgument(
                name="odometry_source",
                default_value="ground_truth",
                description="Odometry source (ground_truth or wheel encoders).",
                choices=["encoders", "ground_truth"],
            ),
            DeclareLaunchArgument(
                name="nav2",
                default_value="bringup",
                description=(
                    "Nav2 backend: 'bringup' (standard nav2_bringup + maps/test.yaml, "
                    "the working recipe), 'scout' (scout_nav2 stack), or 'none'."
                ),
                choices=["bringup", "scout", "none"],
            ),
            DeclareLaunchArgument(
                name="map",
                default_value=default_map,
                description="Map yaml for nav2:=bringup.",
            ),
            DeclareLaunchArgument(
                name="slam",
                default_value="False",
                description="(nav2:=scout only) Run SLAM instead of localization.",
                choices=["True", "False", "true", "false"],
            ),
            DeclareLaunchArgument(
                name="localization",
                default_value="slam_toolbox",
                description="(nav2:=scout only) Localization backend.",
                choices=["amcl", "slam_toolbox"],
            ),
            DeclareLaunchArgument(
                name="spawn_objects",
                default_value="true",
                description="Spawn M2 detection-target objects (fire extinguisher, chair).",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                name="nav2_delay",
                default_value="8.0",
                description="Seconds to wait after Gazebo before starting Nav2.",
            ),
            DeclareLaunchArgument(
                name="perception",
                default_value="true",
                description=(
                    "Start the M3 perception pipeline (object_detector + "
                    "semantic_map). Requires ultralytics for actual detection."
                ),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                name="publish_map_odom_tf",
                default_value="true",
                description=(
                    "Publish a static identity map->odom TF (the 'rl3' workaround). "
                    "Needed while no localizer publishes map->odom; set false once "
                    "real localization does."
                ),
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
