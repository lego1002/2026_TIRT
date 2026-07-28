#!/usr/bin/env bash
# 機器人 bringup 本體 —— PI 端(headless,不開任何 GUI)。
# 真底盤 + 光達 /scan + 車體模型 TF;SLAM 預設不在這裡跑(在 PC 端 run_slam.sh)。
#
# 平常不用直接跑這支:pi/robot_tmux.sh 的 `bringup` 視窗會呼叫它,而那個視窗由
# 筆電上的 ./robotctl up 遠端拉起。直接執行的時機是「已經 ssh 在 Pi 上,想單獨
# 前景跑一次看輸出」。
#
#   ./run_robot.sh                       # 真底盤 + 光達
#   ./run_robot.sh use_fake_odom:=true   # 沒接底盤,只看模型 + 光達
#   ./run_robot.sh use_slam:=true        # 單機 fallback:連 SLAM 也在 Pi 上跑
# robot_bringup.launch.py 的任何參數都能直接往後帶。
#
# ※ Discovery server 不在這裡啟動(以前會在背景偷偷拉起並把輸出丟去 /tmp)。
#   它改由 tmux 的 `dds` 視窗獨立持有,理由有二:(1) 輸出看得見,壞掉當場知道;
#   (2) 重跑 bringup 時 server 不會跟著重啟 —— 否則 PC 端所有節點都要重新發現一次。
#   要單獨起它:dds/run_discovery_server.sh。
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

# 這支的參數會原封不動交給 ros2 launch,所以旗標形式的東西一律會變成 ros2 自己的
# usage dump("unrecognized arguments: --nav"),完全看不出來錯在哪。在這裡先擋掉並
# 講清楚 —— 尤其 --nav 很容易被誤以為是全域旗標,它其實是筆電端 gcs.sh 的。
for _a in "$@"; do
    case "$_a" in
        -h|--help)
            # 用法直接取自檔頭註解(從第 2 行起,遇到第一行非註解就停),不必兩處維護。
            awk 'NR>1 { if (/^#/) { sub(/^# ?/, ""); print } else { exit } }' "$0"; exit 0 ;;
        --nav|--no-rviz|--down)
            echo "run_robot: 「$_a」是筆電端 ./gcs.sh 的旗標,不是 bringup 參數。" >&2
            echo "           這支是 Pi 端 bringup(真底盤 + 光達 + TF),Nav2 和 SLAM 都跑在 PC。" >&2
            echo "           要跑導航請回到**筆電**:./gcs.sh --down 然後 ./gcs.sh --nav" >&2
            exit 1 ;;
        -*)
            echo "run_robot: 不認識的旗標「$_a」。這支只吃 robot_bringup.launch.py 的 k:=v 參數," >&2
            echo "           例如 use_fake_odom:=true、use_slam:=true、ominibot_port:=/dev/serial0。" >&2
            exit 1 ;;
    esac
done

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi
source "$_here/dds/setup_dds.sh"

# setup_dds.sh 在找不到固定別名 IP 時會刻意不設定任何東西。那種狀態下硬啟動只會
# 得到一台誰也連不上的機器人(而且看起來一切正常),不如當場停下來。
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    echo "run_robot: DDS 沒設定好(見上面訊息),中止。" >&2
    exit 1
fi

# 清掉上一輪殘留的 bringup/driver。孤兒 launch 子行程會在 Ctrl+C 後存活並繼續
# 占著 GPIO-UART,第二個 driver 就會跟它搶同一個 port("multiple access on port"),
# 讀值被打亂 → odom 死掉 → SLAM 每張 scan 都丟 → 地圖爛掉。每次啟動前清乾淨。
# 注意這裡刻意不碰 fastdds discovery,它歸 tmux 的 dds 視窗管。
pkill -f "robot_bringup.launch|ominibot_driver_node|sllidar_node|async_slam_toolbox_node" 2>/dev/null || true
fuser -k /dev/ttyAMA0 2>/dev/null || true
sleep 1

exec ros2 launch car_assemble_description robot_bringup.launch.py "$@"
