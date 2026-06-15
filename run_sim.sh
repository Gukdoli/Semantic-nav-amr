#!/usr/bin/env bash
# run_sim.sh — launch the semantic-nav simulation with this machine's required workarounds.
#
# Usage:
#   conda deactivate            # if conda base is active
#   bash ~/nav2_semantic_ws/run_sim.sh        [extra ros2 launch args]
#   bash ~/nav2_semantic_ws/run_sim.sh slam:=True
#
# Why each line is here (see project memory "sim-startup-env-gotchas"):
#  - CYCLONEDDS_URI: ~/.bashrc (or bash_aliases) pins CycloneDDS to enp118s0,
#    which is down -> every ROS2 node dies. Use loopback instead.
#  - IGN_IP: keep ign-transport local-only so multiple/zombie gazebo instances
#    don't collide (collision -> server "malloc(): invalid size" crash).
#  - __EGL_VENDOR_LIBRARY_FILENAMES: this laptop is Intel+NVIDIA hybrid; ogre2's
#    EGL device platform otherwise picks Intel Mesa which fails "create dri2
#    screen". Forcing the NVIDIA EGL vendor renders cleanly (needs the user in
#    the `render`+`video` groups, already done).
#
# NOTE: the real blocker was NOT rendering — it was libassimp5 5.4 (savoury1 PPA)
# vs DART/ign-physics built for assimp 5.2 -> heap corruption (malloc invalid
# size) on mesh-collision load. Fixed by downgrading+holding libassimp5 to
# 5.2.2~ds0-1. See project memory "gazebo-malloc-assimp-abi".

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='lo' multicast='true'/></Interfaces></General></Domain></CycloneDDS>"
export IGN_IP=127.0.0.1
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

source /opt/ros/humble/setup.bash
source "$HOME/nav2_semantic_ws/install/setup.bash"

# Let gz resolve `package://`->`model://` meshes from ROS share dirs (e.g. the
# RealSense D435i body mesh model://realsense2_description/meshes/d435.dae).
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/humble/share:${IGN_GAZEBO_RESOURCE_PATH}"

# clear leftover gazebo processes (zombies cause malloc crashes on next launch)
pkill -9 -f 'ign gazebo' 2>/dev/null
pkill -9 -f ign-gazebo   2>/dev/null
pkill -9 -f ruby         2>/dev/null
# clear leftover M3 perception nodes (run with a separate process lifetime than
# gazebo; otherwise repeated runs stack duplicate object_detector/semantic_map
# nodes that eat CPU and pollute the graph with same-name node warnings).
pkill -9 -f object_detector_node 2>/dev/null
pkill -9 -f semantic_map_node    2>/dev/null
sleep 1

echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "Launching sim (nav2:=bringup -> standard nav2_bringup + maps/test.yaml)..."
# nav2 backend defaults to 'bringup' (standard nav2_bringup + semantic_nav_bringup/maps/test.yaml,
# the recipe that actually drives). Override e.g. `./run_sim.sh nav2:=scout` or `nav2:=none`.
exec ros2 launch semantic_nav_bringup sim.launch.py "$@"
