#!/usr/bin/env bash
# 模擬版 bringup —— 對應實機的 ./run_robot.sh。
#
#   ./sim/run_sim.sh                                # 預設場地(9x9 比賽場),無故障
#   ./sim/run_sim.sh world:=sim/worlds/simple.yaml  # 3x3 小場地,除錯用
#   ./sim/run_sim.sh sim_laser_yaw:=2.9146          # 注入故障(處方見 sim/faults.md)
#
# sim_bringup.launch.py 的任何參數都能直接往後帶,和 run_robot.sh 的用法一致。
#
# 和實機版的三個差別,都是刻意的:
#   1. 不做 DDS 設定。模擬是單機,兩個節點在同一台電腦上,不需要 Discovery Server、
#      不需要固定別名 IP、不需要 interfaceWhiteList。實機那一整套是為了「Pi 和筆電
#      跨 WiFi 通訊」而存在的,模擬裡沒有那個問題。
#      -> 反過來說:網路類的故障(掉包、卡頓)在模擬裡是用 cmd_dropout_prob /
#         cmd_stall_prob 參數注入的,不是真的丟封包。
#   2. 不 pkill 序列埠、不搶 /dev/ttyAMA0,因為沒有硬體。
#   3. 自動偵測 ROS 發行版(實機寫死 humble,開發機可能是 jazzy)。
set -uo pipefail
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
_repo="$( cd "$_here/.." && pwd )"

for _a in "$@"; do
    case "$_a" in
        -h|--help)
            awk 'NR>1 { if (/^#/) { sub(/^# ?/, ""); print } else { exit } }' "$0"; exit 0 ;;
        --nav|--no-rviz|--down)
            echo "run_sim: 「$_a」是 ./sim/gcs_sim.sh 的旗標,不是 bringup 參數。" >&2
            exit 1 ;;
        -*)
            echo "run_sim: 不認識的旗標「$_a」。這支只吃 sim_bringup.launch.py 的 k:=v 參數。" >&2
            echo "         例如 world:=sim/worlds/simple.yaml、odom_linear_error:=1.3" >&2
            exit 1 ;;
    esac
done

ROS_D=""
for d in humble jazzy iron rolling kilted; do
    [ -f "/opt/ros/$d/setup.bash" ] && ROS_D="$d" && break
done
[ -z "$ROS_D" ] && { echo "run_sim: 找不到 ROS 2。先跑 ./sim/setup_sim.sh" >&2; exit 1; }

set +u
source "/opt/ros/$ROS_D/setup.bash"
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
set -u

if ! ros2 pkg prefix tirt_sim >/dev/null 2>&1; then
    echo "run_sim: 找不到 tirt_sim 套件 —— 還沒安裝。先跑:" >&2
    echo "         ./sim/setup_sim.sh" >&2
    exit 1
fi

# 清掉上一輪殘留。孤兒節點會繼續發 /odom 和 odom->base_link TF,和新的那份打架,
# 現象是車子在 RViz 裡瞬移抽動 —— 看起來像模擬器壞了,其實是跑了兩份。
# pattern 帶前綴去對可執行檔路徑,不要用裸名字:pkill -f 比對整條命令列,
# 裸名字會連「只是提到這個名字」的行程(grep、tail、編輯器、甚至操作者自己的 shell)
# 一起殺掉 —— 實機的 run_robot.sh 就踩過兩次。
pkill -f "sim_bringup\.launch\.py|/fake_base$|/fake_lidar$" 2>/dev/null || true
sleep 0.5

# world 用相對路徑時補成絕對路徑 —— launch 的工作目錄不一定是 repo 根目錄。
args=()
for a in "$@"; do
    case "$a" in
        world:=*)
            p="${a#world:=}"
            [ "${p#/}" = "$p" ] && p="$_repo/$p"
            args+=("world:=$p") ;;
        *) args+=("$a") ;;
    esac
done

exec ros2 launch tirt_sim sim_bringup.launch.py "${args[@]}"
