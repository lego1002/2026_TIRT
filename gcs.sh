#!/usr/bin/env bash
# 地面站(Ground Control Station)—— 筆電端**唯一**要記的指令。
#
#   === 建圖模式(預設):開 SLAM,用鍵盤把車開一圈把地圖畫出來 ===
#   ./gcs.sh                      # 全部拉起來並進入操作畫面
#   ./gcs.sh use_fake_odom:=true  # 帶參數給 Pi 的 bringup(等同 robotctl args + restart)
#   ./gcs.sh --no-rviz            # 不開 RViz(例如只想遙控 / 只想看 log)
#                                 # RViz 是由 slam_pc.launch.py 一起帶出來的,不是另外開的
#
#   === 導航模式:讀存好的地圖,用 Nav2 自己走 ===
#   ./gcs.sh --nav                # 用 maps/201_self_test.yaml
#   ./gcs.sh --nav maze_01        # 用 maps/maze_01.yaml
#
#   ./gcs.sh --down               # 收工:關掉 Pi 端全部程式與本機 session(兩種模式共用)
#
# 兩個模式為什麼不能同時開:SLAM 和 Nav2 的 AMCL 都會發 map->odom TF,同時跑會讓
# TF 樹打架(車子在 RViz 裡鬼影亂跳),而且 slam_toolbox 的即時 /map 會和 map_server
# 讀進來的存檔地圖互相蓋掉。所以 --nav 是「取代」slam 視窗,不是多開一個。
#
# --nav 還會順手把 Pi 上的 teleop 關掉,並且不把它拉起來。teleop 為了餵 driver 的
# watchdog,閒著時也會以 20Hz 持續發零速度到 /cmd_vel,和 Nav2 的指令交錯打進來的
# 結果是車子抽一下停一下 —— 看起來非常像硬體故障,其實只是兩個東西在搶同一個 topic。
# 導航途中想手動介入就在 robot 視窗按 Ctrl-a,自己 `./robotctl up teleop`(但記得
# 用完要再關掉)。
#
# 它做的事:
#   1. 套用本機 DDS profile(固定別名,不必查 IP、不必設 DDS_SERVER)
#   2. 遠端把 Pi 上的 dds / bringup / teleop 拉起來(./robotctl up;--nav 不含 teleop)
#   3. 開一個 tmux session,把「Pi 的即時輸出 + 鍵盤遙控」「PC 端 SLAM 或 Nav2 + RViz」
#      「雜事 shell」各放一個視窗 —— 從此不需要開第二個終端,也不需要 ssh 進 Pi
#      (RViz 是 slam_pc.launch.py / nav2_pc.launch.py 自己帶出來的 GUI 視窗,設定檔
#       已預載,不用手動加 display)
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
MODE=slam          # slam(建圖,預設)| nav(讀存好的地圖跑 Nav2)
NAV_MAP=""         # --nav 的地圖名;空的話由 run_nav2.sh 用它的預設
BRINGUP_ARGS=()

for a in "$@"; do
    case "$a" in
        --nav) MODE=nav ;;
        --no-rviz) USE_RVIZ=0 ;;
        --down)
            "$_here/robotctl" down
            tmux kill-session -t "$SESSION" 2>/dev/null && echo "gcs: 已關閉本機 session"
            pkill -f "async_slam_toolbox_node" 2>/dev/null
            pkill -f "rviz2 .*view_robot.rviz" 2>/dev/null
            # Nav2 那一套:container 不理 SIGTERM(實測會變孤兒繼續吃 CPU),直接 -9。
            pkill -f "rviz2 .*view_nav2.rviz" 2>/dev/null
            pkill -9 -f "component_container_isolated.*nav2_container" 2>/dev/null
            exit 0 ;;
        # 用法直接取自檔頭註解(從第 3 行起,遇到第一行非註解就停),不必兩處維護。
        -h|--help) awk 'NR>2 { if (/^#/) { sub(/^# ?/, ""); print } else { exit } }' "$0"; exit 0 ;;
        # bringup 參數一律長 k:=v。裸名字只有一種可能:--nav 要用的地圖名。
        *:=*) BRINGUP_ARGS+=("$a") ;;
        *)    NAV_MAP="$a" ;;
    esac
done

if [ -n "$NAV_MAP" ] && [ "$MODE" != nav ]; then
    echo "gcs: 「$NAV_MAP」看起來是地圖名,但沒有加 --nav。" >&2
    echo "     要跑導航是:./gcs.sh --nav $NAV_MAP" >&2
    echo "     (bringup 參數的格式是 k:=v,例如 use_fake_odom:=true)" >&2
    exit 1
fi

command -v tmux >/dev/null || { echo "筆電上沒裝 tmux:sudo apt install tmux" >&2; exit 1; }

# 從 tmux 裡面再 attach 一個 session 會被拒絕("sessions should be nested with care"),
# 而且因為是 exec,腳本會當場消失、看起來像什麼都沒發生。裡面要用 switch-client。
enter_session() {
    if [ -n "${TMUX:-}" ]; then
        tmux switch-client -t "$SESSION"
    else
        exec tmux attach -t "$SESSION"
    fi
}

# 已經開著就直接回去,不要重開一份。
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "gcs: session 已存在,直接進入(要重來請先 ./gcs.sh --down)"
    enter_session
    exit 0
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
_share="$_prefix/share/car_assemble_description"
_rebuild="cd ~/ros2_ws && colcon build --packages-select car_assemble_description --symlink-install"
if [ "$USE_RVIZ" = 1 ] && ! grep -q 'use_rviz' "$_share/launch/slam_pc.launch.py" 2>/dev/null; then
    echo "gcs: 警告 —— 安裝的 slam_pc.launch.py 是舊版(沒有 use_rviz),RViz 不會自己出來。" >&2
    echo "     修法:$_rebuild" >&2
    echo "     然後在 slam 視窗 Ctrl-c、按 ↑ Enter 重跑即可(不必整套重開)。" >&2
    echo >&2
fi
# 導航模式的同一個坑,但是會直接失敗而不是安靜地少東西。2026-07-28 的兩個變更都需要
# 重 build:nav2 是新檔案(launch/rviz 是新增的),而參數檔搬到 repo 根目錄的 config/
# 是靠 CMakeLists 的 install(../config) —— CMakeLists 改了就一定要重 build,
# 不像平常改 launch/config 那樣 symlink 直接生效。
if [ "$MODE" = nav ]; then
    _missing=""
    [ -f "$_share/launch/nav2_pc.launch.py" ] || _missing="$_missing launch/nav2_pc.launch.py"
    [ -f "$_share/config/nav2_params.yaml" ]  || _missing="$_missing config/nav2_params.yaml"
    [ -f "$_share/rviz/view_nav2.rviz" ]      || _missing="$_missing rviz/view_nav2.rviz"
    if [ -n "$_missing" ]; then
        echo "gcs: PC 端安裝的 car_assemble_description 缺少 Nav2 需要的檔案:" >&2
        for f in $_missing; do echo "       $f" >&2; done
        echo "     這是「git pull 了但沒重 build」。CMakeLists.txt 這次有改(參數檔搬到" >&2
        echo "     repo 根目錄的 config/),所以一定要重 build 一次:" >&2
        echo "       $_rebuild" >&2
        echo "     另外 Nav2 本身也要裝:sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup" >&2
        exit 1
    fi
    if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
        echo "gcs: PC 端沒裝 Nav2。" >&2
        echo "     sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup" >&2
        exit 1
    fi
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
if [ "$MODE" = nav ]; then
    # 不啟動 teleop(它閒著也會 20Hz 發零速度搶 /cmd_vel),而且如果上一輪還開著就關掉。
    "$_here/robotctl" down teleop >/dev/null 2>&1 || true
    "$_here/robotctl" up dds bringup shell || exit 1
else
    "$_here/robotctl" up || exit 1
fi
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
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "gcs: 建立 tmux session 失敗。手動試一次看錯誤:" >&2
    echo "     bash --rcfile $RC -i" >&2
    exit 1
fi
tmux set-option -t "$SESSION" -g history-limit 20000 >/dev/null
tmux set-option -t "$SESSION" mouse on >/dev/null
# 視窗裡的程式(或 shell 本身)掛掉時保留視窗與畫面,而不是讓它連同錯誤訊息一起消失。
# 沒有這行的話,唯一的視窗一死整個 session 就跟著不見,現象是「跑完什麼都沒有」,
# 完全無從查起。死掉的 pane 會標成 [dead],用 restart 重生即可。
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

# slam / nav 視窗會連 RViz 一起帶出來(slam_pc.launch.py 和 nav2_pc.launch.py 的
# use_rviz 都預設 true),所以這裡不要再另外開一個 rviz2,否則會有兩個視窗搶同一份設定。
# 兩個模式只會有其中一個視窗存在 —— 它們會搶 map->odom TF,見檔頭說明。
_rviz_arg=""
[ "$USE_RVIZ" = 1 ] || _rviz_arg=" use_rviz:=false"
if [ "$MODE" = nav ]; then
    new_win nav "$_here/run_nav2.sh${NAV_MAP:+ $NAV_MAP}$_rviz_arg"
else
    new_win slam "$_here/run_slam.sh$_rviz_arg"
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

if [ "$MODE" = nav ]; then
cat <<EOF

======================================================================
 【導航模式】地圖 = ${NAV_MAP:-201_self_test}(SLAM 沒有啟動,teleop 也沒有)
 視窗:  robot = Pi 的即時輸出   nav = PC 端 Nav2 + RViz   shell = 雜事
 切視窗:  Ctrl-b 0/1/2  或  Ctrl-b n
 收    工:  ./gcs.sh --down

 在 RViz 裡下指令:
   1. 車子畫的位置不對 → 工具列 "2D Pose Estimate",點在真實位置、拖出朝向
      (車放回建圖起跑點的話已經自動對好,可以跳過)
   2. 工具列 "Nav2 Goal" → 在地圖上點目標點、拖出朝向 → 車子自己走
   3. 左下 Navigation 2 面板可以看狀態 / 按 Cancel 中止

 要手動介入:Ctrl-b 切到 robot,Ctrl-a 3 進 Pi 的 shell,./robotctl up teleop
            (用完一定要 ./robotctl down teleop,不然它會一直搶 /cmd_vel)
 調參數:config/nav2_params.yaml —— 改完在 nav 視窗 Ctrl-c、↑ Enter 重跑即可
         調哪些、怎麼調看 notes/nav2_tuning.md
======================================================================

EOF
else
cat <<EOF

======================================================================
 【建圖模式】要改成讀存好的地圖自己走:./gcs.sh --down 然後 ./gcs.sh --nav
 視窗:  robot = Pi 的即時輸出 + 鍵盤遙控   slam = PC 端建圖   shell = 雜事
 切視窗:  Ctrl-b 0/1/2  或  Ctrl-b n
 存 地 圖:  Ctrl-b m           (也可以在 shell 視窗打 ./save_map.sh <名字>)
 收    工:  ./gcs.sh --down

 注意 robot 視窗裡是 Pi 的 tmux,它的 prefix 是 Ctrl-a(不是 Ctrl-b):
   Ctrl-a 0/1/2/3 = 切 dds / bringup / teleop / shell 視窗
   Ctrl-a d       = 離開 Pi 的 tmux 回到這裡(不要用 Ctrl-c)
======================================================================

EOF
fi
sleep 2
enter_session
