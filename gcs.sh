#!/usr/bin/env bash
# 地面站(Ground Control Station)—— 筆電端**唯一**要記的指令。
#
#   ./gcs.sh                      # 全部拉起來並進入操作畫面
#   ./gcs.sh use_fake_odom:=true  # 帶參數給 Pi 的 bringup(等同 robotctl args + restart)
#   ./gcs.sh --no-rviz            # 不開 RViz(例如只想遙控 / 只想看 log)
#                                 # RViz 是由 slam_pc.launch.py 一起帶出來的,不是另外開的
#   ./gcs.sh --down               # 收工:關掉 Pi 端全部程式與本機 session
#
# 它做的事:
#   1. 套用本機 DDS profile(固定別名,不必查 IP、不必設 DDS_SERVER)
#   2. 遠端把 Pi 上的 dds / bringup / teleop 拉起來(./robotctl up)
#   3. 開一個 tmux session,把「Pi 的即時輸出 + 鍵盤遙控」「PC 端 SLAM + RViz」
#      「雜事 shell」各放一個視窗 —— 從此不需要開第二個終端,也不需要 ssh 進 Pi
#      (RViz 是 slam_pc.launch.py 自己帶出來的 GUI 視窗,設定檔已預載,不用手動加 display)
#
# 遙控鍵盤為什麼在 Pi 上跑:teleop 節點跑在 Pi,/cmd_vel 就不必穿越 WiFi。
# 你在這裡按的鍵是走 ssh(TCP,可靠)進去的,WiFi 抖一下最多是按鍵晚幾十毫秒到,
# 而不是像以前那樣 /cmd_vel 掉包 → driver 的 watchdog 把底盤歸零 → 車子一頓一頓。
set -uo pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
source "$_here/net/tirt_net.conf"

SESSION=gcs
RC="${XDG_RUNTIME_DIR:-/tmp}/tirt_gcs_rc.sh"
USE_RVIZ=1
BRINGUP_ARGS=()

for a in "$@"; do
    case "$a" in
        --no-rviz) USE_RVIZ=0 ;;
        --down)
            "$_here/robotctl" down
            tmux kill-session -t "$SESSION" 2>/dev/null && echo "gcs: 已關閉本機 session"
            pkill -f "async_slam_toolbox_node" 2>/dev/null
            pkill -f "rviz2 .*view_robot.rviz" 2>/dev/null
            exit 0 ;;
        # 用法直接取自檔頭註解(從第 3 行起,遇到第一行非註解就停),不必兩處維護。
        -h|--help) awk 'NR>2 { if (/^#/) { sub(/^# ?/, ""); print } else { exit } }' "$0"; exit 0 ;;
        *) BRINGUP_ARGS+=("$a") ;;
    esac
done

command -v tmux >/dev/null || { echo "筆電上沒裝 tmux:sudo apt install tmux" >&2; exit 1; }

# 已經開著就直接回去,不要重開一份。
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "gcs: session 已存在,直接進入(要重來請先 ./gcs.sh --down)"
    exec tmux attach -t "$SESSION"
fi

# --- 0. 前置檢查:PC 端套件有沒有 build 到最新 -----------------------------
# ROS 的 setup.bash 在 `set -u` 下會踩到未定義變數而中斷,所以只在這段關掉。
set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
set -u

if ! _prefix="$(ros2 pkg prefix car_assemble_description 2>/dev/null)"; then
    echo "gcs: 找不到 car_assemble_description —— PC 端還沒 build。" >&2
    echo "     cd ~/ros2_ws && colcon build --packages-select car_assemble_description --symlink-install" >&2
    exit 1
fi
# 「pull 了但沒重 build」是最容易踩、又最難看出來的狀況:launch 檔跑的是安裝過的舊副本,
# RViz 就只是安靜地不出現,畫面上沒有任何錯誤可看。這裡直接把它講清楚。
_installed_launch="$_prefix/share/car_assemble_description/launch/slam_pc.launch.py"
if [ "$USE_RVIZ" = 1 ] && ! grep -q 'use_rviz' "$_installed_launch" 2>/dev/null; then
    echo "gcs: 警告 —— 安裝的 slam_pc.launch.py 是舊版(沒有 use_rviz),RViz 不會自己出來。" >&2
    echo "     修法:cd ~/ros2_ws && colcon build --packages-select car_assemble_description --symlink-install" >&2
    echo "     然後在 slam 視窗 Ctrl-c、按 ↑ Enter 重跑即可(不必整套重開)。" >&2
    echo >&2
fi

# --- 1. 本機 DDS ----------------------------------------------------------
source "$_here/dds/setup_dds.sh"
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    echo "gcs: DDS 沒設定好(見上面訊息),中止。" >&2
    exit 1
fi

# --- 2. 遠端拉起 Pi -------------------------------------------------------
if [ ${#BRINGUP_ARGS[@]} -gt 0 ]; then
    "$_here/robotctl" args "${BRINGUP_ARGS[*]}" || exit 1
fi
"$_here/robotctl" up || exit 1
if [ ${#BRINGUP_ARGS[@]} -gt 0 ]; then
    # up 只會啟動「還沒在跑」的視窗;參數有變就一定要重跑 bringup 才吃得到。
    "$_here/robotctl" restart bringup || exit 1
fi

# --- 3. 本機 tmux ---------------------------------------------------------
cat > "$RC" <<EOF
[ -f /etc/bash.bashrc ] && . /etc/bash.bashrc
[ -f "\$HOME/.bashrc" ] && . "\$HOME/.bashrc"
. /opt/ros/humble/setup.bash
[ -f "\$HOME/ros2_ws/install/setup.bash" ] && . "\$HOME/ros2_ws/install/setup.bash"
. "$_here/dds/setup_dds.sh"
cd "$_here"
EOF

new_win() {  # new_win <名稱> [要送出的指令]
    local name="$1" cmd="${2:-}"
    if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx "$name"; then :
    else tmux new-window -d -t "$SESSION" -n "$name" -c "$_here" "bash --rcfile '$RC' -i"; sleep 0.3; fi
    # 用 send-keys 而不是直接把指令當視窗程式:指令會留在該 shell 的歷史裡,
    # Ctrl-C 之後按 ↑ Enter 就能重跑,方便一段一段排查。
    [ -n "$cmd" ] && tmux send-keys -t "$SESSION:$name" "$cmd" C-m
}

tmux new-session -d -s "$SESSION" -n placeholder -c "$_here" "bash --rcfile '$RC' -i"
tmux set-option -t "$SESSION" -g history-limit 20000 >/dev/null
tmux set-option -t "$SESSION" mouse on >/dev/null

# slam 視窗會連 RViz 一起帶出來(slam_pc.launch.py 的 use_rviz 預設 true),
# 所以這裡不要再另外開一個 rviz2,否則會有兩個視窗搶同一份設定。
if [ "$USE_RVIZ" = 1 ]; then
    new_win slam "$_here/run_slam.sh"
else
    new_win slam "$_here/run_slam.sh use_rviz:=false"
fi
new_win shell ""
# robot 視窗放最後才建,attach 時預設停在它上面(Pi 的輸出 + 遙控都在這裡)
new_win robot "$_here/robotctl attach"
tmux kill-window -t "$SESSION:placeholder" 2>/dev/null

# 存圖快速鍵:Ctrl-b m → 在 shell 視窗存一張以時間命名的地圖
tmux bind-key -T prefix m run-shell \
    "tmux send-keys -t $SESSION:shell '$_here/save_map.sh maze_\$(date +%H%M%S)' C-m; tmux select-window -t $SESSION:shell"

if [ "$USE_RVIZ" = 1 ] && [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    echo "gcs: 警告 —— 沒有 DISPLAY,RViz 開不起來。要純文字操作請用 ./gcs.sh --no-rviz。"
fi

cat <<EOF

======================================================================
 視窗:  robot = Pi 的即時輸出 + 鍵盤遙控   slam = PC 端建圖   shell = 雜事
 切視窗:  Ctrl-b 0/1/2  或  Ctrl-b n
 存 地 圖:  Ctrl-b m           (也可以在 shell 視窗打 ./save_map.sh <名字>)
 收    工:  ./gcs.sh --down

 注意 robot 視窗裡是 Pi 的 tmux,它的 prefix 是 Ctrl-a(不是 Ctrl-b):
   Ctrl-a 0/1/2/3 = 切 dds / bringup / teleop / shell 視窗
   Ctrl-a d       = 離開 Pi 的 tmux 回到這裡(不要用 Ctrl-c)
======================================================================

EOF
sleep 2
exec tmux attach -t "$SESSION"
