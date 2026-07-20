#!/usr/bin/env bash
# One-click SLAM -- LAPTOP SIDE.
# Runs async_slam_toolbox on the PC (not the Pi): the Pi streams /scan + odom TF over
# DDS, the PC does the heavy scan matching and publishes map->odom + /map back.
# This offloads the CPU-bound scan matching that the Pi 4 couldn't keep up with
# (queue-full scan drops -> rotational map smear). See car_assemble_description/launch/
# slam_pc.launch.py for the full rationale.
#
# Prereqs on the PC:
#   sudo apt install ros-humble-slam-toolbox
#   colcon build --packages-select car_assemble_description   # to get config/ + this launch
#
# Typical PC-side terminals: ./run_slam.sh (here) + ./run_rviz.sh + teleop + ./save_map.sh
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi
source "$_here/dds/setup_dds.sh"

# Guard: kill a stale slam from a previous run so map->odom isn't published twice.
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
sleep 1

exec ros2 launch car_assemble_description slam_pc.launch.py "$@"
