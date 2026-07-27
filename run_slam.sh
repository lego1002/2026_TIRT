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
# 平常不用直接跑這支:./gcs.sh 會在它的 `slam` 視窗裡呼叫。單獨跑也完全沒問題。
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi

# DDS 角色由本機的固定別名 IP 自動判斷,不再需要 DDS_SERVER=<pi_ip>。
# 別名沒掛上時 setup_dds.sh 會報錯且不設定任何東西 —— 那就別硬跑,不然只會得到
# 一個永遠等不到 /scan 的 slam_toolbox(看起來活著,實際上什麼都收不到)。
source "$_here/dds/setup_dds.sh"
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    echo "run_slam: DDS 沒設定好(見上面訊息),中止。" >&2
    exit 1
fi

# Guard: kill a stale slam from a previous run so map->odom isn't published twice.
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
sleep 1

exec ros2 launch car_assemble_description slam_pc.launch.py "$@"
