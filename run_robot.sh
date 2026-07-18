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

exec ros2 launch car_assemble_description robot_bringup.launch.py "$@"
