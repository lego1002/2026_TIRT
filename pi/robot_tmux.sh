#!/usr/bin/env bash
# 在「樹莓派」上管理機器人各段程式的 tmux session。
#
# 平常不會直接跑這支 —— 筆電上的 ./robotctl 會透過 ssh 呼叫它。要在 Pi 本機用也可以:
#
#   pi/robot_tmux.sh up [視窗...]      # 預設全部;已在跑的不動
#   pi/robot_tmux.sh restart bringup   # 只重跑某一段
#   pi/robot_tmux.sh down [視窗...]
#   pi/robot_tmux.sh status
#   pi/robot_tmux.sh attach
#
# 為什麼用 tmux 而不是 systemd:
#   刻意選擇「手動啟動、看得見原生輸出」。每一段各佔一個視窗,attach 進去看到的
#   就跟自己 ssh 進來手跑一模一樣;要排查某一段時直接在那個視窗 Ctrl-C,按 ↑ Enter
#   就重跑「那一段」,其他段不受影響(尤其 dds 視窗不能跟著重啟,不然 PC 端全部要重連)。
#   指令是用 send-keys 打進互動式 bash 的,所以它真的存在於該 shell 的歷史裡,↑ 找得到。
#
# 每個視窗的 shell 都預先 source 好 ROS + ~/ros2_ws + dds/setup_dds.sh(見 _RC 檔),
# 所以在任何視窗裡直接下 ros2 指令都能用。
set -uo pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
REPO="$( cd "$_here/.." && pwd )"
source "$REPO/net/tirt_net.conf"

S="${TIRT_PI_SESSION:-tirt}"
RC="${XDG_RUNTIME_DIR:-/tmp}/tirt_window_rc.sh"
ARGS_FILE="$HOME/.tirt_robot_args"

WINDOWS=(dds bringup teleop shell)

# 各視窗要跑的指令(shell 視窗留空,只給你一個 source 好環境的 prompt)。
win_cmd() {
    case "$1" in
        dds)     echo "$REPO/dds/run_discovery_server.sh" ;;
        bringup) echo "$REPO/run_robot.sh $(cat "$ARGS_FILE" 2>/dev/null)" ;;
        teleop)  echo "ros2 run ominibot_driver mecanum_teleop" ;;
        shell)   echo "" ;;
        *)       return 1 ;;
    esac
}

die() { echo "robot_tmux: $*" >&2; exit 1; }

command -v tmux >/dev/null || die "Pi 上沒裝 tmux:sudo apt install tmux"

write_rc() {
    cat > "$RC" <<EOF
# 由 pi/robot_tmux.sh 產生的視窗初始化檔,每次啟動都會覆寫。
[ -f /etc/bash.bashrc ] && . /etc/bash.bashrc
[ -f "\$HOME/.bashrc" ] && . "\$HOME/.bashrc"
. /opt/ros/humble/setup.bash
[ -f "\$HOME/ros2_ws/install/setup.bash" ] && . "\$HOME/ros2_ws/install/setup.bash"
. "$REPO/dds/setup_dds.sh"
cd "$REPO"
EOF
}

# 固定別名 IP 是整套通訊的前提;沒有它就算節點都起來了,PC 端也永遠連不上。
# 開機後 wifi 要幾秒才就緒,所以這裡等,而不是直接失敗。
wait_for_alias() {
    local i
    for i in $(seq 1 30); do
        ip -4 -o addr show scope global | grep -q " ${TIRT_PI_IP}/" && return 0
        [ "$i" = 1 ] && echo "robot_tmux: 等待固定別名 ${TIRT_PI_IP} 就緒..."
        sleep 1
    done
    echo "robot_tmux: 錯誤 —— 30 秒內沒等到 ${TIRT_PI_IP}。" >&2
    echo "robot_tmux:   檢查 wifi 是否連上,以及是否跑過 sudo ./net/install_pi_network.sh" >&2
    return 1
}

ensure_session() {
    tmux has-session -t "$S" 2>/dev/null && return 0
    write_rc
    tmux new-session -d -s "$S" -n placeholder -c "$REPO" "bash --rcfile '$RC' -i"
    # 把 prefix 改成 C-a:筆電端的 gcs.sh 也是 tmux(預設 C-b),巢狀時才不會打架。
    tmux set-option -t "$S" prefix C-a >/dev/null
    tmux set-option -t "$S" -g history-limit 20000 >/dev/null
    tmux set-option -t "$S" mouse on >/dev/null
    # 程式(或視窗的 shell)掛掉時保留視窗和畫面上的錯誤訊息,而不是讓它整個消失。
    # 這是排查的前提:沒有它,discovery server 死掉只會看到視窗不見,錯誤訊息一起帶走。
    tmux set-option -t "$S" remain-on-exit on >/dev/null
    echo "robot_tmux: 已建立 session '$S'(prefix = Ctrl-a)"
}

win_exists() { tmux list-windows -t "$S" -F '#W' 2>/dev/null | grep -qx "$1"; }

# 視窗裡是不是真的有東西在跑(而不是 Ctrl-C 之後只剩一個 bash prompt,或整個 pane 已死)
win_busy() {
    local info dead c
    info="$(tmux list-panes -t "$S:$1" -F '#{pane_dead} #{pane_current_command}' 2>/dev/null | head -1)"
    [ -n "$info" ] || return 1
    dead="${info%% *}"; c="${info#* }"
    [ "$dead" = "1" ] && return 1          # remain-on-exit 留下的屍體不算在跑
    [ -n "$c" ] && [ "$c" != "bash" ] && [ "$c" != "sh" ]
}

# pane 死掉後 send-keys 沒有用(沒有 shell 在收),必須先 respawn。
win_dead() {
    [ "$(tmux list-panes -t "$S:$1" -F '#{pane_dead}' 2>/dev/null | head -1)" = "1" ]
}

start_win() {
    local w="$1" cmd
    cmd="$(win_cmd "$w")" || die "未知的視窗名:$w(可用:${WINDOWS[*]})"

    if win_exists "$w"; then
        if win_busy "$w"; then
            echo "robot_tmux: [$w] 已在執行,略過(要重跑用 restart $w)"
            return 0
        fi
        if win_dead "$w"; then
            # pane 已死(remain-on-exit 把屍體留著給你看錯誤),裡面沒有 shell 收得到
            # send-keys,必須先重生一個。
            echo "robot_tmux: [$w] 上次異常結束(畫面已保留),重新開一個 shell"
            write_rc
            tmux respawn-window -k -t "$S:$w" -c "$REPO" "bash --rcfile '$RC' -i"
            sleep 0.3
        else
            # 視窗還在、shell 也在,只是程式停了 —— 重新把指令打進去,保留捲動歷史
            echo "robot_tmux: [$w] 視窗閒置,重新啟動"
        fi
    else
        write_rc
        tmux new-window -d -t "$S" -n "$w" -c "$REPO" "bash --rcfile '$RC' -i"
        # 等 bash 起來再送鍵,否則按鍵會掉。
        sleep 0.3
        echo "robot_tmux: [$w] 已建立"
    fi

    if [ -n "$cmd" ]; then
        # send-keys 而不是把指令當成視窗的 command:這樣指令會進入該 shell 的歷史,
        # Ctrl-C 之後按 ↑ Enter 就能重跑同一段。
        tmux send-keys -t "$S:$w" "$cmd" C-m
        echo "robot_tmux: [$w] > $cmd"
    fi

    # placeholder 視窗只在建立 session 那一瞬間存在,有真的視窗了就收掉。
    win_exists placeholder && tmux kill-window -t "$S:placeholder" 2>/dev/null
    return 0
}

stop_win() {
    local w="$1"
    win_exists "$w" || { echo "robot_tmux: [$w] 不存在"; return 0; }
    tmux kill-window -t "$S:$w"
    echo "robot_tmux: [$w] 已關閉"
}

cmd_up() {
    wait_for_alias || exit 1
    ensure_session
    local list=("$@")
    [ ${#list[@]} -eq 0 ] && list=("${WINDOWS[@]}")
    local w
    for w in "${list[@]}"; do
        start_win "$w"
        # discovery server 要先站穩,後面的節點才找得到它。
        [ "$w" = dds ] && sleep 1.5
    done
}

cmd_down() {
    tmux has-session -t "$S" 2>/dev/null || { echo "robot_tmux: session '$S' 沒在跑"; return 0; }
    if [ $# -eq 0 ]; then
        tmux kill-session -t "$S"
        echo "robot_tmux: 已關閉整個 session '$S'"
        # tmux kill 不保證子孫行程都收掉;底盤的 UART 被卡住會讓下次啟動讀到亂碼。
        pkill -f "robot_bringup.launch|ominibot_driver_node|sllidar_node|mecanum_teleop" 2>/dev/null
        pkill -f "fastdds discovery|fast-discovery-server" 2>/dev/null
        fuser -k /dev/ttyAMA0 2>/dev/null
        return 0
    fi
    local w
    for w in "$@"; do stop_win "$w"; done
}

cmd_restart() {
    [ $# -eq 0 ] && die "restart 要指定視窗:${WINDOWS[*]}"
    ensure_session
    local w
    for w in "$@"; do
        win_cmd "$w" >/dev/null || die "未知的視窗名:$w"
        if win_exists "$w"; then
            # respawn -k 砍掉舊行程並在同一個視窗開一個乾淨的 shell
            write_rc
            tmux respawn-window -k -t "$S:$w" -c "$REPO" "bash --rcfile '$RC' -i"
            sleep 0.3
            local cmd; cmd="$(win_cmd "$w")"
            [ -n "$cmd" ] && tmux send-keys -t "$S:$w" "$cmd" C-m
            echo "robot_tmux: [$w] 已重跑"
        else
            start_win "$w"
        fi
    done
}

cmd_status() {
    if ! tmux has-session -t "$S" 2>/dev/null; then
        echo "session '$S': 沒在跑"
        return 1
    fi
    printf '%-10s %-10s %s\n' 視窗 狀態 目前行程
    local w c
    for w in "${WINDOWS[@]}"; do
        if win_exists "$w"; then
            c="$(tmux list-panes -t "$S:$w" -F '#{pane_current_command}' 2>/dev/null | head -1)"
            if win_busy "$w"; then
                printf '%-10s %-10s %s\n' "$w" "執行中" "$c"
            else
                printf '%-10s %-10s %s\n' "$w" "閒置" "$c (程式已停)"
            fi
        else
            printf '%-10s %-10s %s\n' "$w" "無" "-"
        fi
    done
    echo
    echo "固定別名 ${TIRT_PI_IP}: $(ip -4 -o addr show scope global | grep -q " ${TIRT_PI_IP}/" && echo 有 || echo '沒有 (!)')"
    echo "啟動參數 ($ARGS_FILE): $(cat "$ARGS_FILE" 2>/dev/null || echo '(空,用 launch 預設)')"
}

cmd_log() {
    local w="${1:-bringup}" n="${2:-200}"
    tmux has-session -t "$S" 2>/dev/null || die "session '$S' 沒在跑"
    win_exists "$w" || die "視窗 [$w] 不存在"
    tmux capture-pane -p -S "-$n" -t "$S:$w"
}

case "${1:-}" in
    up)      shift; cmd_up "$@" ;;
    down)    shift; cmd_down "$@" ;;
    restart) shift; cmd_restart "$@" ;;
    status)  shift; cmd_status "$@" ;;
    log)     shift; cmd_log "$@" ;;
    attach)  exec tmux attach -t "$S" ;;
    *)
        cat <<EOF
用法: $(basename "$0") <指令> [視窗...]

  up [視窗...]       啟動(預設全部:${WINDOWS[*]});已在跑的不動
  restart <視窗...>  只重跑指定的那幾段
  down [視窗...]     關閉;不給視窗名就關掉整個 session 並清乾淨殘留行程
  status             各視窗死活 + 別名 IP + 啟動參數
  log [視窗] [行數]  抓該視窗最近的輸出(預設 bringup 200 行)
  attach             進入 session(prefix = Ctrl-a)
EOF
        exit 1 ;;
esac
