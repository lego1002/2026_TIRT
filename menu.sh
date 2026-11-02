#!/usr/bin/env bash
# TIRT 機器人控制台 —— 在「樹莓派」上跑。用 MobaXterm ssh 進 Pi 之後:
#
#   cd ~/2026_TIRT && ./menu.sh
#
# 單機架構(2026-10-26 起):建圖、導航、RViz 全部在 Pi 上算,Windows 電腦只是
# 「終端機 + 顯示 RViz 的螢幕」(MobaXterm 內建 X server)。理由:
#   - 比賽規則不准用遠端電腦運算,比賽當天本來就只能全部在 Pi 上跑。
#   - 不用再管兩台機器的 DDS、別名 IP、discovery server;WiFi 斷了機器人照跑。
#   - 實驗室和自己筆電都是 Windows + MobaXterm,不必另外裝 Ubuntu。
# 選單關掉(q)或 ssh 斷線,背景的建圖 / 導航都會繼續跑(它們在 tmux session 裡);
# 再打一次 ./menu.sh 就回來了。要回到舊的「Pi + Ubuntu 筆電」雙機模式:rm ~/.tirt_solo
set -o pipefail
REPO="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
T="$REPO/pi/robot_tmux.sh"
source "$REPO/net/tirt_net.conf"
S="${TIRT_PI_SESSION:-tirt}"

# ---- 第一次執行:切成單機模式 ------------------------------------------------
if [ ! -f "$HOME/.tirt_solo" ]; then
    echo "這個選單是「全部在 Pi 上跑」的單機模式,第一次使用要切換一次。"
    echo "(切換後筆電端的 ./gcs.sh 雙機模式就不能用;要切回去:rm ~/.tirt_solo)"
    read -r -p "切換成單機模式? [Y/n] " _a
    case "$_a" in n|N) exit 0 ;; esac
    # 舊的雙機 session 是用另一個 DDS 設定開的,和單機的節點互相看不到,先整個關掉。
    if tmux has-session -t "$S" 2>/dev/null; then
        echo "先關掉舊的雙機 session..."
        "$T" down
    fi
    touch "$HOME/.tirt_solo"
    echo "已切換。(車上 OLED 要重開機一次,或 sudo systemctl restart tirt-oled,才會改用單機設定)"
    echo
fi

source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
source "$REPO/dds/setup_dds.sh" >/dev/null

# Ctrl-C 只停掉前景那一段(例如遙控),不要把選單一起關掉。
# 必須是 ':' 而不是 '':被忽略('')的信號會遺傳給子行程,遙控就再也 Ctrl-C 不掉了。
trap ':' INT

busy() { "$T" busy "$1" 2>/dev/null; }
mark() { if busy "$1"; then printf '\e[32m●執行中\e[0m'; else printf '○ 沒開 '; fi; }
pause() { echo; read -r -p "按 Enter 回選單..." _; }

power_text() {
    local v; v="$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)"
    case "$v" in
        0x0) printf '\e[32mOK\e[0m' ;;
        "")  printf '?' ;;
        # 低 4 bit 是「現在」,16~19 bit 是「開機以來發生過」:
        #   0x1 欠壓  0x2 降頻  0x4 被限速  0x8 溫度過高(軟限制 80°C)
        # 不能一律說欠壓:2026-10-26 實測 0x80000 是過熱,原本的文字把人帶去查電源。
        *)   local now=$(( v & 0xF )) past=$(( (v >> 16) & 0xF )) msg=""
             (( now & 0x1 ))  && msg+="現在欠壓! "
             (( now & 0x8 ))  && msg+="現在過熱! "
             (( now & 0x6 ))  && msg+="現在降速中! "
             (( past & 0x1 && !(now & 0x1) )) && msg+="開機後曾欠壓 "
             (( past & 0x8 && !(now & 0x8) )) && msg+="開機後曾過熱 "
             [ -z "$msg" ] && msg="開機後曾降速 "
             if (( now )); then printf '\e[31m%s(%s)\e[0m' "$msg" "$v"
             else printf '\e[33m%s(%s)\e[0m' "$msg" "$v"; fi ;;
    esac
}

need_bringup() {
    busy bringup && return 0
    echo "底盤+光達還沒開。請先選 1(建圖)或 4(導航)。"
    return 1
}

do_mapping() {
    if busy nav; then echo "關掉導航(建圖和導航不能同時開)..."; "$T" down nav; fi
    # run_robot.sh 一啟動就會 pkill 殘留的 slam_toolbox,所以 slam 要等 bringup 過了那一步才開,
    # 不然剛起來的 slam 會被它當成殘留砍掉。
    if ! busy bringup; then "$T" up bringup; echo "等底盤+光達起來..."; sleep 5; fi
    "$T" up slam
    echo
    echo "建圖開始了。開 RViz(選 5)大約 10~15 秒後會看到地圖。"
    echo "接著選 2 用鍵盤慢慢開車把迷宮走一遍,走完選 3 存地圖。"
}

do_teleop() {
    # 導航模式也准遙控:nav2 沒有目標時完全不發 /cmd_vel(2026-10-26 實測 6 秒 0 則),
    # 不會打架;而且推車走一段正是讓 AMCL 收斂的方法。只有「遙控開著又按 Nav2 Goal」
    # 會打架 —— teleop 閒著也以 20Hz 發零速度,會和 nav2 的指令交錯。
    if busy nav; then
        echo "※ 導航模式:遙控開著的時候不要按 Nav2 Goal(兩邊會搶指令,車子會抽動)。"
        echo "   要讓車子自己走,先 Ctrl-C 結束遙控,再去 RViz 點目標。"
    fi
    need_bringup || return
    cat <<'EOF'
------------------------------------------------------------
 鍵盤遙控            u  i  o      i/,  = 前進/後退
                     j  k  l      j/l  = 往左/往右平移
                     m  ,  .      k/空白 = 停
 a/d = 原地轉   w/s = 移動速度 加/減   q/e = 旋轉速度 加/減
 Ctrl-C = 結束遙控,回到選單(車會停)
------------------------------------------------------------
EOF
    ros2 run ominibot_driver mecanum_teleop
}

do_save() {
    if ! busy slam; then echo "建圖沒在跑,沒有地圖可以存。先選 1。"; return; fi
    local def="map_$(date +%m%d_%H%M)" name
    read -r -p "地圖名字 [Enter = $def]: " name
    name="${name:-$def}"
    if [[ ! "$name" =~ ^[A-Za-z0-9_-]+$ ]]; then echo "名字只能用英文、數字、_ 和 -。"; return; fi
    "$REPO/save_map.sh" "$name" && echo "存好了:maps/$name.yaml(之後選 4 就能選到它)"
}

do_nav() {
    local maps=() f i choice
    for f in "$REPO"/maps/*.yaml; do
        [ -e "$f" ] || continue
        case "$(basename "$f")" in sim_*) continue ;; esac
        maps+=("$(basename "$f" .yaml)")
    done
    if [ ${#maps[@]} -eq 0 ]; then echo "maps/ 裡沒有地圖。先選 1 建圖、3 存圖。"; return; fi
    echo "選一張地圖:"
    for i in "${!maps[@]}"; do printf '  %2d) %s\n' $((i + 1)) "${maps[$i]}"; done
    read -r -p "編號: " choice
    if ! [[ "$choice" =~ ^[0-9]+$ ]] || (( choice < 1 || choice > ${#maps[@]} )); then
        echo "沒有這個編號。"; return
    fi
    echo "${maps[$((choice - 1))]}" > "$HOME/.tirt_nav_map"
    if busy slam; then echo "關掉建圖(建圖和導航不能同時開)..."; "$T" down slam; fi
    if busy nav;  then echo "關掉正在跑的導航,換地圖..."; "$T" down nav; fi
    "$T" up bringup nav
    echo "$REPO/car_assemble_description/rviz/view_nav2.rviz" > "$HOME/.tirt_rviz_cfg"
    busy rviz && "$T" restart rviz >/dev/null
    echo
    echo "導航啟動中(Pi 上大約要 20~40 秒)。開 RViz(選 5)後:"
    echo "  1. 車子在地圖上的位置不對 → 工具列 '2D Pose Estimate' 點車子真正的位置、拖出朝向"
    echo "  2. 選 9 讓它原地定位,直到綠色箭頭縮成一小團"
    echo "  3. 工具列 'Nav2 Goal' 點目標、拖出朝向 → 車子自己走過去"
}

do_rviz() {
    if busy rviz; then "$T" down rviz; echo "RViz 已關。"; return; fi
    # 在 tmux 裡開的選單,DISPLAY 是「tmux 第一次啟動那條 ssh」的;MobaXterm 重連之後
    # 那個號碼就失效了(2026-10-26 實測:RViz 對著死掉的 localhost:10.0,X 連線斷 → segfault,
    # Windows 上什麼都沒出現,而新連線其實是 localhost:11.0)。所以檢查 X 轉送的 port
    # (6000+N)還有沒有人在聽,失效就改用最新那條連線的。
    local n port
    n="${DISPLAY:-}"; n="${n#localhost:}"; n="${n%%.*}"
    if [ -z "$n" ] || ! ss -tln 2>/dev/null | grep -q "127.0.0.1:$((6000 + n)) "; then
        port="$(ss -tln 2>/dev/null | grep -o '127.0.0.1:60[0-9][0-9] ' | cut -d: -f2 | sort -n | tail -1)"
        if [ -n "$port" ]; then
            export DISPLAY="localhost:$((port - 6000)).0"
            echo "原本的畫面轉送(${n:+localhost:$n.0})已失效,改用目前的連線 $DISPLAY。"
        else
            unset DISPLAY
        fi
    fi
    if [ -z "${DISPLAY:-}" ]; then
        echo "這條連線沒有畫面轉送(DISPLAY 是空的)。"
        echo "MobaXterm:Session 設定 → Advanced SSH settings → 勾 'X11-forwarding',重新連線。"
        echo "(右上角的 X server 圖示要是亮的)"
        return
    fi
    echo "$DISPLAY" > "$HOME/.tirt_display"
    if busy nav; then
        echo "$REPO/car_assemble_description/rviz/view_nav2.rviz" > "$HOME/.tirt_rviz_cfg"
    else
        echo "$REPO/car_assemble_description/rviz/view_robot.rviz" > "$HOME/.tirt_rviz_cfg"
    fi
    "$T" up rviz >/dev/null
    echo "RViz 開啟中,視窗會出現在 Windows 桌面上(第一次要等幾秒)。"
    echo "※ RViz 也是 Pi 在算,會吃 CPU。正式比賽時不要開;這條 ssh 斷了它也會跟著關。"
}

laptop_text() {
    if [ -f "$HOME/.tirt_laptop_rviz" ]; then printf '\e[36m開(Ubuntu 筆電可看)\e[0m'; else printf '關(只有 Pi 自己)'; fi
}

do_laptop() {
    # 讓 Ubuntu 筆電直接跑 RViz(筆電的 CPU 畫圖,Pi 只送資料)。規則允許:筆電只看不算,
    # 而且主辦方說過起點感應器前可以用 RViz 點目標(見 memory §12)。
    # 切換要「全部重開」:每個 ROS 節點只在啟動當下讀一次 DDS 設定,新舊設定的節點互相看不到。
    if [ -f "$HOME/.tirt_laptop_rviz" ]; then
        echo "目前:筆電可看 RViz。要關掉(回到只有 Pi,比賽保底設定)嗎?"
    else
        echo "目前:只有 Pi。要打開「Ubuntu 筆電看 RViz」嗎?"
        echo "(Pi 會多跑一個 Discovery Server;WiFi 斷了 Pi 上的建圖/導航照樣跑,只是筆電看不到)"
    fi
    if tmux has-session -t "$S" 2>/dev/null; then
        echo "⚠ 切換會把正在跑的底盤 / 建圖 / 導航全部關掉,之後要重新選 1 或 4。"
        echo "  (還沒存的地圖會不見 —— 要先存就選 n,回去選 3)"
    fi
    read -r -p "確定切換? [y/N] " a
    case "$a" in y|Y) ;; *) echo "沒改。"; return ;; esac
    tmux has-session -t "$S" 2>/dev/null && "$T" down >/dev/null
    if [ -f "$HOME/.tirt_laptop_rviz" ]; then rm -f "$HOME/.tirt_laptop_rviz"; else touch "$HOME/.tirt_laptop_rviz"; fi
    source "$REPO/dds/setup_dds.sh" >/dev/null
    echo "已切換:筆電看 RViz = $(laptop_text)"
    if [ -f "$HOME/.tirt_laptop_rviz" ]; then
        echo
        echo "接下來:先在這裡選 1(建圖)或 4(導航),然後在 Ubuntu 筆電的終端機打:"
        echo "    cd ~/2026_TIRT && ./run_rviz.sh          # 建圖時"
        echo "    cd ~/2026_TIRT && ./run_rviz.sh nav      # 導航時(有 Nav2 Goal 和定位狀態字)"
        echo "筆電要跟 Pi 在同一個 WiFi(筆電要有 10.77.0.1),或用網路線(筆電 10.78.0.1)。"
        echo "※ 用網路線的話,要「先插線、再選 1 或 4」—— 節點只在啟動時看有哪些網卡。"
    fi
    # OLED 服務也要換成同一套 DDS 設定,不然它看不到電池 / 光達。服務是用 lego 身分跑、
    # Restart=always,所以不需要 sudo:對它 Ctrl-C(SIGINT,會畫收尾畫面),5 秒後 systemd 自己重開。
    pkill -INT -u "$USER" -f '^/usr/bin/python3 /opt/ros/[^ ]*/bin/ros2 run ominibot_driver oled_status' 2>/dev/null || true
}

do_health() {
    echo "檢查中(約 8 秒)..."
    # 量測交給一個 python 節點一次做完(理由見 tools/health_check.py 檔頭)。
    local out scan odom batt
    out="$(timeout 20 python3 "$REPO/tools/health_check.py" 5 2>/dev/null)"
    scan="$(sed -n 's/^scan_hz=//p' <<< "$out")"
    odom="$(sed -n 's/^odom_hz=//p' <<< "$out")"
    batt="$(sed -n 's/^batt_v=//p' <<< "$out")"
    [ -n "$batt" ] && batt="$batt V"
    echo
    printf '  樹莓派供電   %b\n' "$(power_text)"
    printf '  電池電壓     %s\n' "${batt:-讀不到(底盤沒開?)}"
    printf '  光達 /scan   %s   (正常約 10 Hz)\n' "${scan:-沒有資料} ${scan:+Hz}"
    printf '  里程 /odom   %s   (正常約 20 Hz)\n' "${odom:-沒有資料} ${odom:+Hz}"
    printf '  CPU 溫度     %s\n' "$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2)"
    printf '  CPU 負載     %s   (4 核,超過 4 代表忙不過來)\n' "$(cut -d' ' -f1-3 /proc/loadavg)"
}

radius_text() {
    # 沒有 ~/.tirt_nav_radius = 用 config/nav2_params.yaml 的比賽值
    local rr ir
    if [ -f "$HOME/.tirt_nav_radius" ] && read -r rr ir < "$HOME/.tirt_nav_radius"; then
        printf '\e[33m房間 %s/%s\e[0m' "$rr" "$ir"
    else
        printf '\e[32m比賽 0.09/0.18\e[0m'
    fi
}

do_radius() {
    # 安全圈 = robot_radius(車心離牆小於它就當撞到,決定窄處過不過得去)
    #        + inflation_radius(牆邊「不想靠近」的帶子,只影響路線偏好,不擋路)。
    # 存在 ~/.tirt_nav_radius,run_nav2.sh 開導航時套用;導航正開著就順便即時改。
    echo "目前安全圈:$(radius_text)   (robot_radius / inflation_radius,單位公尺)"
    echo "  1) 比賽  0.09 / 0.18   44 cm 迷宮走道用這個(碰牆就失敗,留足空間)"
    echo "  2) 房間  0.07 / 0.12   家裡門縫、桌腳很窄時用(車身可能很貼牆)"
    echo "  3) 自訂"
    local c rr ir
    read -r -p "選擇: " c
    c="$(sed 's/[[:space:]]//g; y/０１２３/0123/' <<< "$c")"
    case "$c" in
        1) rr=0.09; ir=0.18; rm -f "$HOME/.tirt_nav_radius" ;;
        2) rr=0.07; ir=0.12; echo "$rr $ir" > "$HOME/.tirt_nav_radius" ;;
        3) read -r -p "robot_radius (建議 0.06~0.10): " rr
           read -r -p "inflation_radius (要比 robot_radius 大,建議 0.10~0.20): " ir
           if ! [[ "$rr" =~ ^0\.[0-9]+$ && "$ir" =~ ^0\.[0-9]+$ ]] \
              || ! python3 -c "import sys; r,i=map(float,sys.argv[1:]); sys.exit(not (0.04<=r<=0.15 and r<i<=0.4))" "$rr" "$ir"; then
               echo "數字不合理(robot_radius 0.04~0.15,inflation_radius 要比它大、最多 0.4)。沒改。"; return
           fi
           echo "$rr $ir" > "$HOME/.tirt_nav_radius" ;;
        *) echo "沒改。"; return ;;
    esac
    echo "已設定 $(radius_text),下次開導航(選 4)會套用。"
    if busy nav; then
        echo "導航正在跑,即時套用中(Pi 忙的時候每項要等幾秒到幾十秒)..."
        local cm ok=1
        for cm in global_costmap/global_costmap local_costmap/local_costmap; do
            timeout 60 ros2 param set "/$cm" robot_radius "$rr" >/dev/null || ok=0
            timeout 60 ros2 param set "/$cm" inflation_layer.inflation_radius "$ir" >/dev/null || ok=0
        done
        if (( ok )); then echo "已即時套用。"; else echo "即時套用有一項沒成功;重開導航(選 4)就會生效。"; fi
    fi
}

do_log() {
    local w
    read -r -p "看哪一段? [bringup / slam / nav / rviz,Enter = bringup]: " w
    w="${w:-bringup}"
    "$T" log "$w" 60 | grep -v '^$' | tail -40
}

while true; do
    echo
    echo "================ TIRT 機器人控制台(全部在 Pi 上跑)================"
    printf '  底盤+光達 %b   建圖 %b   導航 %b   RViz %b   供電 %b\n' \
        "$(mark bringup)" "$(mark slam)" "$(mark nav)" "$(mark rviz)" "$(power_text)"
    printf '  安全圈 %b   筆電 RViz %b\n' "$(radius_text)" "$(laptop_text)"
    echo "-------------------------------------------------------------------"
    echo "  1) 開始建圖            (底盤 + 光達 + SLAM)"
    echo "  2) 鍵盤遙控            (Ctrl-C 回選單)"
    echo "  3) 存地圖"
    echo "  4) 開始導航            (選一張存好的地圖)"
    echo "  5) 開 / 關 RViz        (畫面顯示在 Windows 上)"
    echo "  6) 健康檢查            (供電、電池、光達、里程)"
    echo "  7) 看某一段的輸出      (出問題時看錯誤訊息)"
    echo "  8) 進入 tmux(進階)    (Ctrl-a d 回來)"
    echo "  9) 原地定位            (導航中,2D Pose Estimate 之後用)"
    echo "  r) 調整安全圈          (比賽 / 房間 / 自訂)"
    echo "  l) 筆電看 RViz 開/關   (Ubuntu 筆電自己畫 RViz,Pi 省 CPU)"
    echo "  0) 全部關掉"
    echo "  q) 離開選單            (背景程式繼續跑)"
    read -r -p "選擇: " c || exit 0
    # 中文輸入法開著時打出來的是全形「９」,會落到「沒有這個選項」;順便去掉空白。
    c="$(sed 's/[[:space:]]//g; y/０１２３４５６７８９ｑＱｒＲｌＬ/0123456789qQrRlL/' <<< "$c")"
    echo
    case "$c" in
        1) do_mapping ;;
        2) do_teleop; continue ;;
        3) do_save ;;
        4) do_nav ;;
        5) do_rviz ;;
        6) do_health ;;
        7) do_log ;;
        8) "$T" attach; continue ;;
        9) if busy nav; then python3 "$REPO/tools/amcl_settle.py"; else echo "導航沒開。先選 4。"; fi ;;
        0) "$T" down; echo "全部關掉了(光達馬達也停了)。" ;;
        r|R) do_radius ;;
        l|L) do_laptop ;;
        q|Q) echo "選單已離開。再打 ./menu.sh 就回來。"; exit 0 ;;
        *) echo "沒有這個選項。"; continue ;;
    esac
    pause
done
