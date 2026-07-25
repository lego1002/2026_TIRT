#!/usr/bin/env bash
# One-click RViz2 "which way is the robot facing?" view -- LAPTOP SIDE.
#
# This is the *pre-SLAM* sanity view. Differences from run_rviz.sh:
#   * Fixed Frame is `odom`, not `map`  -> works with plain `./run_robot.sh`, no SLAM needed
#   * TopDownOrtho projection            -> no perspective distortion, so angles are readable
#                                           (judging angles in the default Orbit view is how the
#                                            167-degree laser_yaw error went unnoticed for so long)
#   * Big Axes markers on base_link and laser_frame -> red = +X, green = +Y, blue = +Z
#   * No Map display                     -> nothing to confuse the picture
#
# Use it to confirm, by eye:
#   i  -> robot travels along base_link +X  (the RED arrow)
#   j  -> robot strafes along base_link +Y  (the GREEN arrow)
#   a  -> robot spins counter-clockwise
# and to see the ~167 degree offset between base_link's red axis and laser_frame's red axis.
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi

if [ -z "$DDS_SERVER" ]; then
    echo "run_rviz_orient: WARNING -- DDS_SERVER unset; run as  DDS_SERVER=<pi_ip> ./run_rviz_orient.sh"
fi
source "$_here/dds/setup_dds.sh"

exec rviz2 -d "$_here/car_assemble_description/rviz/orientation_check.rviz"
