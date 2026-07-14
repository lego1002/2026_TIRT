#!/usr/bin/env bash
# One-click RViz2 viewer -- LAPTOP SIDE.
# Opens RViz2 with the saved view (Grid, RobotModel, LaserScan, Map, Odometry, TF).
# Needs car_assemble_description built locally so RobotModel meshes (package:// URIs) resolve.
#
# Fixed Frame is `map`: if the view is blank at first, SLAM on the Pi takes ~10-15 s to
# create the `map` frame -- either wait, or temporarily set Fixed Frame to `base_link`.
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi
source "$_here/dds/setup_dds.sh"

exec rviz2 -d "$_here/car_assemble_description/rviz/view_robot.rviz"
