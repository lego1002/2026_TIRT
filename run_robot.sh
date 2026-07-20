#!/usr/bin/env bash
# One-click robot bringup -- PI SIDE (headless).
# Starts the real chassis driver + lidar + SLAM + model TF via robot_bringup.launch.py.
# RViz is NOT opened here; run ./run_rviz.sh on the laptop to view.
#
#   ./run_robot.sh                       # real chassis + lidar + SLAM (default)
#   ./run_robot.sh use_fake_odom:=true   # no chassis board attached, just model + lidar
#   ./run_robot.sh use_slam:=false       # skip map building
# Any robot_bringup.launch.py arg can be passed through.
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi
source "$_here/dds/setup_dds.sh"

# Guard: kill any stale bringup/driver from a previous run before launching.
# Orphaned launch children survive Ctrl+C and keep holding the GPIO-UART serial
# port; a second driver then fights over it ("multiple access on port"), reads
# get corrupted, odom dies, and SLAM drops every scan -> broken map. Clearing
# them here makes every start clean.
pkill -f "robot_bringup.launch|ominibot_driver_node|sllidar_node|async_slam_toolbox_node" 2>/dev/null || true
fuser -k /dev/ttyAMA0 2>/dev/null || true
sleep 1

exec ros2 launch car_assemble_description robot_bringup.launch.py "$@"
