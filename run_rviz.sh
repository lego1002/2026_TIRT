#!/usr/bin/env bash
# One-click RViz2 viewer -- LAPTOP SIDE.
# Opens RViz2 with the saved view (Grid, RobotModel, LaserScan, Map, Odometry, TF).
# Needs car_assemble_description built locally so RobotModel meshes (package:// URIs) resolve.
#
# Fixed Frame is `map`: if the view is blank at first, SLAM on the Pi takes ~10-15 s to
# create the `map` frame -- either wait, or temporarily set Fixed Frame to `base_link`.
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

for _d in humble jazzy; do [ -f "/opt/ros/$_d/setup.bash" ] && { source "/opt/ros/$_d/setup.bash"; break; }; done
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi

# DDS 角色由本機的固定別名 IP 自動判斷,不再需要 DDS_SERVER=<pi_ip>。
source "$_here/dds/setup_dds.sh"
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    echo "run_rviz: DDS 沒設定好(見上面訊息),中止。" >&2
    exit 1
fi

# ./run_rviz.sh nav → 導航用的畫面(Nav2 Goal、costmap、粒子、車頂定位狀態字)。
# 單機模式下 Pi 開了 menu.sh 的「l) 筆電看 RViz」後,筆電就是用這支看。
_cfg=view_robot.rviz
case "${1:-}" in nav|--nav) _cfg=view_nav2.rviz ;; esac
exec rviz2 -d "$_here/car_assemble_description/rviz/$_cfg"
