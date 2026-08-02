#!/usr/bin/env bash
# One-click map saver. Saves the live SLAM /map to <name>.pgm + <name>.yaml.
# Uses a longer save_map_timeout so the latched /map sample is reliably received
# (the map_saver_cli default ~2s often misses it and errors "Failed to spin map subscription").
#
#   ./save_map.sh                 # -> maps/map_<default>.pgm/.yaml
#   ./save_map.sh maze_01         # -> maps/maze_01.pgm/.yaml
#   ./save_map.sh ~/foo/bar       # absolute/relative path also works
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

# 實機(Pi 與比賽筆電)都是 humble;開發/模擬機可能是別的發行版,所以找不到
# humble 時往下找一個可用的,而不是直接失敗。以前這行是寫死的 `source
# /opt/ros/humble/setup.bash`,在 jazzy 的模擬機上會被 set -e 當場中止 ——
# 症狀就是 gcs_sim.sh 的 Ctrl-b m 按下去只閃一行「No such file or directory」。
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    for _d in jazzy iron rolling kilted; do
        [ -f "/opt/ros/$_d/setup.bash" ] && { source "/opt/ros/$_d/setup.bash"; break; }
    done
fi
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi

# TIRT_SIM=1 由 sim/gcs_sim.sh 設定 —— 模擬是單機,不需要 Discovery Server /
# 固定別名 IP(那一整套是為了「Pi 和筆電跨 WiFi」而存在的)。和 run_slam.sh /
# run_nav2.sh 同一個逃生門,存地圖在模擬裡才會是同一個指令。
if [ "${TIRT_SIM:-0}" = 1 ]; then
    echo "save_map: TIRT_SIM=1 -> 單機模擬模式,略過 DDS 設定。"
else
    source "$_here/dds/setup_dds.sh"
    if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
        echo "save_map: DDS 沒設定好(見上面訊息),中止 —— 否則 map_saver 只會空等到逾時。" >&2
        exit 1
    fi
fi

name="${1:-my_map}"
# If a bare name (no slash) is given, drop it under the repo's maps/ dir.
case "$name" in
  */*) out="$name" ;;
  *)   mkdir -p "$_here/maps"; out="$_here/maps/$name" ;;
esac

echo "Saving map to ${out}.pgm / ${out}.yaml ..."
exec ros2 run nav2_map_server map_saver_cli -f "$out" --ros-args -p save_map_timeout:=10.0
