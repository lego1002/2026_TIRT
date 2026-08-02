#!/usr/bin/env bash
# 模擬地面站 —— 對應實機的 ./gcs.sh,操作方式刻意做成一模一樣。
#
#   === 建圖模式(預設):開 SLAM,用鍵盤把車開一圈把地圖畫出來 ===
#   ./sim/gcs_sim.sh
#   ./sim/gcs_sim.sh world:=sim/worlds/simple.yaml   # 換場地
#   ./sim/gcs_sim.sh sim_laser_yaw:=2.9146           # 注入故障(見 sim/faults.md)
#
#   === 導航模式:讀存好的地圖,用 Nav2 自己走 ===
#   ./sim/gcs_sim.sh --nav                # 用 maps/sim_maze.yaml(沒有的話自動產生)
#   ./sim/gcs_sim.sh --nav my_slam_map    # 用自己 SLAM 掃出來的圖
#
#   ./sim/gcs_sim.sh --down               # 收工
#
# 和實機 ./gcs.sh 的對應關係:
#
#   實機                              模擬
#   ------------------------------    ------------------------------
#   robotctl up(ssh 到 Pi 開 tmux)   sim 視窗直接跑 run_sim.sh    <- 沒有第二台機器
#   robot 視窗 = Pi 的即時輸出 + 遙控  sim / teleop 兩個視窗         <- 拆開,因為不必經 ssh
#   slam 視窗 = run_slam.sh           slam 視窗 = run_slam.sh       <- 同一支腳本
#   nav  視窗 = run_nav2.sh           nav  視窗 = run_nav2.sh       <- 同一支腳本
#   Ctrl-b m 存地圖                    Ctrl-b m 存地圖               <- 同一個快速鍵
#
# 兩個模式為什麼不能同時開:和實機一樣 —— slam_toolbox 和 Nav2 的 AMCL 都會發
# map->odom TF,同時跑 TF 樹會打架。--nav 是「取代」slam 視窗,不是多開一個。
set -uo pipefail
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
_repo="$( cd "$_here/.." && pwd )"

SESSION=simgcs
RC="${XDG_RUNTIME_DIR:-/tmp}/tirt_simgcs_rc.sh"
MODE=slam
NAV_MAP=""
USE_RVIZ=1
SIM_ARGS=()

for a in "$@"; do
    case "$a" in
        --nav) MODE=nav ;;
        --no-rviz) USE_RVIZ=0 ;;
        --down)
            tmux kill-session -t "$SESSION" 2>/dev/null && echo "已關閉模擬 session"
            pkill -f "sim_bringup\.launch\.py|/fake_base$|/fake_lidar$" 2>/dev/null
            pkill -f "async_slam_toolbox_node" 2>/dev/null
            pkill -f "rviz2 .*view_(robot|nav2)\.rviz" 2>/dev/null
            pkill -f "/mecanum_teleop$" 2>/dev/null
            # nav2 的 container 不理 SIGTERM(實機實測會變孤兒繼續吃 CPU),直接 -9
            pkill -9 -f "component_container_isolated.*nav2_container" 2>/dev/null
            exit 0 ;;
        -h|--help) awk 'NR>1 { if (/^#/) { sub(/^# ?/, ""); print } else { exit } }' "$0"; exit 0 ;;
        *:=*) SIM_ARGS+=("$a") ;;
        *)    NAV_MAP="$a" ;;
    esac
done

if [ -n "$NAV_MAP" ] && [ "$MODE" != nav ]; then
    echo "gcs_sim: 「$NAV_MAP」看起來是地圖名,但沒有加 --nav。" >&2
    echo "         要跑導航是:./sim/gcs_sim.sh --nav $NAV_MAP" >&2
    exit 1
fi

command -v tmux >/dev/null || { echo "沒裝 tmux:sudo apt install tmux" >&2; exit 1; }

ROS_D=""
for d in humble jazzy iron rolling kilted; do
    [ -f "/opt/ros/$d/setup.bash" ] && ROS_D="$d" && break
done
[ -z "$ROS_D" ] && { echo "gcs_sim: 找不到 ROS 2。先跑 ./sim/setup_sim.sh" >&2; exit 1; }

set +u
source "/opt/ros/$ROS_D/setup.bash"
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
set -u

if ! ros2 pkg prefix tirt_sim >/dev/null 2>&1; then
    echo "gcs_sim: 找不到 tirt_sim —— 還沒安裝。先跑:  ./sim/setup_sim.sh" >&2
    exit 1
fi

enter_session() {
    if [ -n "${TMUX:-}" ]; then tmux switch-client -t "$SESSION"
    else exec tmux attach -t "$SESSION"; fi
}
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "gcs_sim: session 已存在,直接進入(要重來請先 ./sim/gcs_sim.sh --down)"
    enter_session; exit 0
fi

# --- 導航模式:確保有地圖 -------------------------------------------------
# 沒指定就用模擬場地直接產生一張「幾何完美」的地圖。這是模擬獨有的能力,
# 也是最有價值的除錯手段:用完美地圖跑 Nav2,就能把「地圖爛」和「Nav2 參數錯」
# 這兩個在實機上永遠糾纏在一起的原因徹底分開。
if [ "$MODE" = nav ]; then
    if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
        echo "gcs_sim: 沒裝 Nav2。sudo apt install ros-$ROS_D-navigation2 ros-$ROS_D-nav2-bringup" >&2
        exit 1
    fi
    if [ -z "$NAV_MAP" ]; then
        NAV_MAP=sim_maze
        _world="$_here/worlds/tirt_maze.yaml"
        for a in "${SIM_ARGS[@]:-}"; do
            case "$a" in world:=*) _world="${a#world:=}"
                [ "${_world#/}" = "$_world" ] && _world="$_repo/$_world" ;;
            esac
        done
        if [ ! -f "$_repo/maps/$NAV_MAP.yaml" ]; then
            echo "gcs_sim: 產生完美地圖 maps/$NAV_MAP …"
            ( cd "$_repo" && python3 "$_here/tirt_sim/make_map.py" "$_world" "maps/$NAV_MAP" ) || exit 1
        fi
    fi
fi

# --- tmux -----------------------------------------------------------------
cat > "$RC" <<EOF
[ -f /etc/bash.bashrc ] && . /etc/bash.bashrc
[ -f "\$HOME/.bashrc" ] && . "\$HOME/.bashrc"
. "/opt/ros/$ROS_D/setup.bash"
[ -f "\$HOME/ros2_ws/install/setup.bash" ] && . "\$HOME/ros2_ws/install/setup.bash"
# 模擬是單機,不需要 DDS Discovery Server / 固定別名 IP。實機那一整套是為了
# 「Pi 和筆電跨 WiFi」而存在的。TIRT_SIM=1 讓 run_slam.sh / run_nav2.sh 略過該檢查。
export TIRT_SIM=1
cd "$_repo"
EOF

new_win() {
    local name="$1" cmd="${2:-}"
    tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx "$name" || {
        tmux new-window -d -t "$SESSION" -n "$name" -c "$_repo" "bash --rcfile '$RC' -i"
        sleep 0.3
    }
    # 用 send-keys 而不是把指令當視窗程式:指令留在該 shell 的歷史裡,
    # Ctrl-C 後按 ↑ Enter 就能重跑單一部分,方便一段一段排查。和實機同樣的設計。
    [ -n "$cmd" ] && tmux send-keys -t "$SESSION:$name" "$cmd" C-m
}

tmux new-session -d -s "$SESSION" -n placeholder -c "$_repo" "bash --rcfile '$RC' -i"
tmux has-session -t "$SESSION" 2>/dev/null || {
    echo "gcs_sim: 建立 tmux session 失敗。手動試一次看錯誤: bash --rcfile $RC -i" >&2; exit 1; }
# prefix 明確指定成 C-b,不吃使用者的全域設定 —— 下面印的說明和存地圖快速鍵都寫
# Ctrl-b,而使用者的 ~/.tmux.conf 很可能把全域 prefix 改成了 C-a。實機的 gcs.sh
# 同樣處理(那邊還多一層:Pi 的 tirt session 是 C-a,靠這個和外層錯開)。
tmux set-option -t "$SESSION" prefix C-b >/dev/null
tmux set-option -t "$SESSION" -u prefix2 >/dev/null 2>&1
tmux bind-key -T prefix C-b send-prefix >/dev/null
# history-limit 是 session 選項,原本多寫了一個 -g 變成設全域,會覆蓋掉使用者
# ~/.tmux.conf 裡調高的值。拿掉 -g。
tmux set-option -t "$SESSION" history-limit 20000 >/dev/null
tmux set-option -t "$SESSION" mouse on >/dev/null
# 程式掛掉時保留視窗和畫面,否則唯一的視窗一死整個 session 就消失,
# 現象是「跑完什麼都沒有」,完全無從查起。
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

_rviz_arg=""; [ "$USE_RVIZ" = 1 ] || _rviz_arg=" use_rviz:=false"

# map -> sim_world 的接法要跟著模式走,否則 RViz 裡的「真值幽靈」會畫在錯的位置:
#   建圖:slam_toolbox 把 map 原點定在車子開機的位置  -> start
#   導航:map 原點是 make_map.py 產生地圖的世界原點    -> identity
_tf_mode=start; [ "$MODE" = nav ] && _tf_mode=identity
new_win sim "$_here/run_sim.sh world_tf_mode:=$_tf_mode ${SIM_ARGS[*]:-}"
if [ "$MODE" = nav ]; then
    # Nav2 模式不開 teleop:teleop 閒著也會 20Hz 發零速度餵 watchdog,
    # 和 Nav2 的指令交錯打進來會讓車子抽一下停一下。實機也是同樣的坑。
    new_win nav "sleep 3; $_repo/run_nav2.sh $NAV_MAP$_rviz_arg"
else
    new_win slam "sleep 3; $_repo/run_slam.sh$_rviz_arg"
    # 模擬沒有 cmd_linear_scale 那個 6.5 倍的標定誤差,所以 teleop 的預設
    # 0.6 m/s 在這裡是真的 0.6 m/s —— 對 44cm 的走廊太快了,壓到 0.25。
    new_win teleop "sleep 4; ros2 run ominibot_driver mecanum_teleop --ros-args -p linear_speed:=0.25 -p angular_speed:=1.0"
fi
new_win shell ""
tmux kill-window -t "$SESSION:placeholder" 2>/dev/null

tmux bind-key -T prefix m run-shell \
    "tmux send-keys -t $SESSION:shell '$_repo/save_map.sh sim_\$(date +%H%M%S)' C-m; tmux select-window -t $SESSION:shell"

if [ "$MODE" = nav ]; then
cat <<EOF

======================================================================
 【模擬 · 導航模式】地圖 = $NAV_MAP
 視窗:  sim = 模擬底盤+光達   nav = Nav2 + RViz   shell = 雜事
 切視窗:  Ctrl-b 0/1/2   收工:  ./sim/gcs_sim.sh --down

 在 RViz 裡:工具列 "Nav2 Goal" 點目標點、拖出朝向 -> 車子自己走
 車子起始位置就是地圖原點,所以不用先做 2D Pose Estimate。

 模擬獨有、拿來學習用的 topic:
   ros2 topic echo /sim/odom_error     里程計漂移了多少(真車上你看不到這個)
   ros2 topic echo /sim/collision      有沒有撞牆(規則:碰牆即當次失敗)
======================================================================

EOF
else
cat <<EOF

======================================================================
 【模擬 · 建圖模式】
 視窗:  sim = 模擬底盤+光達   slam = SLAM + RViz   teleop = 鍵盤   shell = 雜事
 切視窗:  Ctrl-b 0/1/2/3      存地圖:  Ctrl-b m
 收    工:  ./sim/gcs_sim.sh --down

 遙控(切到 teleop 視窗):
     u i o      左前 前 右前          a / d : 原地左轉 / 右轉
     j k l      左移 停 右移          w / s : 加減速度
     m , .      左後 後 右後          空白  : 停

 模擬獨有、拿來學習用的 topic:
   ros2 topic echo /sim/odom_error     里程計漂移了多少(真車上你看不到這個)
   ros2 topic echo /sim/collision      有沒有撞牆
======================================================================

EOF
fi
sleep 2
enter_session
