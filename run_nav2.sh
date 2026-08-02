#!/usr/bin/env bash
# One-click Nav2 自主導航 -- LAPTOP SIDE(和 run_slam.sh 二選一,不能同時跑)。
#
# 差別很簡單:
#   ./run_slam.sh    邊走邊建圖(slam_toolbox),用來「畫出」地圖
#   ./run_nav2.sh    讀已經存好的地圖(AMCL 定位 + Nav2 導航),用來「在地圖上跑」
# 兩者都會發 map->odom TF,同時開會讓 TF tree 打架,所以這支會先把 slam 殺掉。
#
# 用法:
#   ./run_nav2.sh                       # 用 maps/201_self_test.yaml
#   ./run_nav2.sh maze_01               # 用 maps/maze_01.yaml(裸名字 → 找 maps/)
#   ./run_nav2.sh ~/foo/bar.yaml        # 也吃絕對/相對路徑
#   ./run_nav2.sh 201_self_test use_rviz:=false     # 額外參數原封不動傳給 launch
#
# 前提(PC 端,做一次就好):
#   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
#   colcon build --packages-select car_assemble_description   # 取得 config/nav2_params.yaml
#
# ★ 開跑前務必先關掉 Pi 上的鍵盤 teleop:./robotctl down teleop
#   teleop_node 為了餵 driver 的 watchdog,是「每迴圈重發目前 Twist」的設計,閒著時
#   會以 20Hz 持續發零速度。它和 nav2 的指令交錯打進 /cmd_vel,車子只會抽一下停一下,
#   而且症狀看起來非常像硬體故障。這支腳本會偵測並提醒,但不會擅自去關遠端的東西。
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

# 旗標一律擋掉。這支的第一個裸參數是「地圖名」,所以 `--nav` 不擋的話會被當成地圖名,
# 然後得到一句「找不到地圖 maps/--nav.yaml」—— 對的錯誤,但答錯了問題。
for _a in "$@"; do
    case "$_a" in
        -h|--help)
            # 用法直接取自檔頭註解(從第 2 行起,遇到第一行非註解就停),不必兩處維護。
            awk 'NR>1 { if (/^#/) { sub(/^# ?/, ""); print } else { exit } }' "$0"; exit 0 ;;
        --nav)
            echo "run_nav2: 這支本身就是導航,不用再加 --nav(那是筆電端 ./gcs.sh 的旗標)。" >&2
            echo "          地圖直接用裸名字帶:./run_nav2.sh 201_self_test" >&2
            exit 1 ;;
        --no-rviz)
            echo "run_nav2: 這支的關 RViz 寫法是 launch 參數:./run_nav2.sh 201_self_test use_rviz:=false" >&2
            exit 1 ;;
        -*)
            echo "run_nav2: 不認識的旗標「$_a」。用法:./run_nav2.sh [地圖名] [k:=v ...]" >&2
            exit 1 ;;
    esac
done

# 實機(Pi 與比賽筆電)都是 humble;開發/模擬機可能是別的發行版,所以找不到
# humble 時往下找一個可用的,而不是直接失敗。
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    for _d in jazzy iron rolling kilted; do
        [ -f "/opt/ros/$_d/setup.bash" ] && { source "/opt/ros/$_d/setup.bash"; break; }
    done
fi
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi

# TIRT_SIM=1 由 sim/gcs_sim.sh 設定 —— 模擬是單機,不需要 Discovery Server /
# 固定別名 IP / interfaceWhiteList(那一整套是為了「Pi 和筆電跨 WiFi」而存在的)。
# 這個逃生門讓模擬直接跑這支腳本本身,而不是複製一份 —— 你在模擬裡練的操作,
# 比賽當天是同一個指令。
if [ "${TIRT_SIM:-0}" = 1 ]; then
    echo "run_nav2: TIRT_SIM=1 -> 單機模擬模式,略過 DDS 設定。"
else
    # DDS 角色由本機的固定別名 IP 自動判斷。別名沒掛上時 setup_dds.sh 會報錯且不設定
    # 任何東西 —— 那就別硬跑,不然只會得到一整包永遠收不到 /scan 的 nav2 節點
    # (lifecycle 會 active,看起來很健康,實際上什麼都收不到)。
    source "$_here/dds/setup_dds.sh"
    if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
        echo "run_nav2: DDS 沒設定好(見上面訊息),中止。" >&2
        exit 1
    fi
fi

# --- 解析地圖引數 -----------------------------------------------------------
# 第一個引數如果長得像 launch 參數(含 :=)就不當成地圖名,直接沿用預設地圖。
map_arg="201_self_test"
if [ $# -gt 0 ] && [[ "$1" != *":="* ]]; then
    map_arg="$1"; shift
fi

case "$map_arg" in
  /*|./*|../*|*/*) map_file="$map_arg" ;;                    # 有路徑成分 → 照用
  *.yaml)          map_file="$_here/maps/$map_arg" ;;         # 裸檔名 → maps/
  *)               map_file="$_here/maps/$map_arg.yaml" ;;    # 裸名字 → maps/<name>.yaml
esac
map_file="$(cd "$(dirname "$map_file")" && pwd)/$(basename "$map_file")"   # → 絕對路徑

# 本機沒這張圖時,自動去 Pi 上找一份回來。
#
# 為什麼需要這段:`/maps` 在 .gitignore 裡,所以地圖**不會**跟著 git 同步;而
# save_map.sh 存在「你執行它的那台機器」。實際流程是「在 Pi 上存圖 → 在 PC 上跑
# Nav2」,而 map_server 跑在 PC —— 兩邊的 maps/ 內容天生就不一樣。2026-07-28 就是
# 這樣卡住的:Pi 上有 201_self_test,PC 上只有另一張,run_nav2.sh 直接說找不到。
#
# 只在「本機缺這個檔」時才複製,所以不可能蓋掉本機任何東西;對 Pi 是唯讀。
# 模擬模式沒有 Pi 可以抓(TIRT_SIM=1),跳過整段 —— 否則會白等一次 ssh 逾時。
if [ ! -f "$map_file" ] && [ "${TIRT_SIM:-0}" != 1 ] && [ -f "$_here/net/tirt_net.conf" ]; then
    source "$_here/net/tirt_net.conf"
    # 自己就是 Pi 的話不用抓(在 Pi 上單機除錯時會走到這裡)。
    if ! ip -4 -o addr show scope global 2>/dev/null | grep -q " ${TIRT_PI_IP}/"; then
        _base="$(basename "$map_file" .yaml)"
        _pi="${TIRT_SSH_USER}@${TIRT_PI_IP}"
        echo "run_nav2: 本機沒有 ${_base},試著從 Pi(${TIRT_PI_IP})複製一份..."
        mkdir -p "$_here/maps"
        if scp -o ConnectTimeout=5 -o BatchMode=yes -q \
               "${_pi}:${TIRT_PI_REPO}/maps/${_base}.yaml" \
               "${_pi}:${TIRT_PI_REPO}/maps/${_base}.pgm" \
               "$_here/maps/" 2>/dev/null; then
            echo "run_nav2: 已複製 ${_base}.yaml + .pgm 到 maps/"
        else
            echo "run_nav2: 從 Pi 複製失敗(Pi 上可能也沒有這張圖,或 ssh 不通)。" >&2
            echo "          Pi 上有哪些圖:ssh ${_pi} 'ls ~/2026_TIRT/maps/*.yaml'" >&2
        fi
    fi
fi

if [ ! -f "$map_file" ]; then
    echo "run_nav2: 找不到地圖 $map_file" >&2
    echo "本機現有的地圖:" >&2
    ls -1 "$_here/maps/"*.yaml 2>/dev/null | xargs -r -n1 basename >&2 || echo "  (maps/ 是空的)" >&2
    echo "注意 /maps 有進 .gitignore,git pull 不會帶地圖過來;在哪台機器存的圖就只在那台。" >&2
    exit 1
fi

# --- 排除會互相打架的東西 ---------------------------------------------------
# SIGTERM 先問,不走再 SIGKILL。實測 nav2 的 component_container 會**不理 SIGTERM**
# 而變成孤兒(2026-07-28:一個殘留的 container 活了 20 分鐘,把 Pi 的 CPU 吃掉,
# 導致下一次啟動的 nav2 卡在啟動階段、整整 46 秒印不出任何一行 log —— 症狀看起來
# 像設定壞了,其實是資源被搶),所以一定要確認它真的死了才繼續。
kill_pattern() {
    local pat="$1" what="$2" i
    pgrep -f "$pat" >/dev/null 2>&1 || return 0
    echo "run_nav2: 偵測到殘留的 $what,先關掉。"
    pkill -f "$pat" 2>/dev/null || true
    for i in 1 2 3 4 5; do
        sleep 1
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
    done
    echo "run_nav2: $what 不理 SIGTERM,改用 SIGKILL。"
    pkill -9 -f "$pat" 2>/dev/null || true
    sleep 1
}

# 1) 本機殘留的 slam_toolbox:它和 AMCL 會搶著發 map->odom。
kill_pattern "async_slam_toolbox_node" "slam_toolbox(會和 AMCL 搶 map->odom TF)"
# 2) 本機殘留的 nav2:重複的 controller/planner 會同時發 /cmd_vel。
#    pattern 刻意寫得這麼長:單純用 "nav2_container" 會命中任何 command line 裡出現
#    這個字串的行程 —— 包括操作者自己正在跑的 grep/tail/編輯器,曾經因此把自己的
#    shell 殺掉。pkill 不會殺自己,但會殺別的無辜行程。
kill_pattern "component_container_isolated.*nav2_container" "nav2 container"

# 3) 網路上「別台機器」的 slam_toolbox。上面的 pkill 只管得到本機,但 DDS 是跨機的:
#    2026-07-28 就踩到 —— PC 上還開著 run_slam.sh,它的 /map(邊建邊長,168×142)和
#    本機 map_server 讀進來的存檔地圖(168×104)同時餵給 AMCL,而且兩邊都在發
#    map->odom。nav2 不會報錯,只是定位莫名其妙地爛。
#    這裡在 nav2 自己的 map_server 還沒起來之前問一次:此刻有人發 /map,就是別人。
if command -v ros2 >/dev/null 2>&1; then
    _pubs="$(timeout 8 ros2 topic info /map 2>/dev/null | sed -n 's/^Publisher count: //p')"
    if [ -n "${_pubs:-}" ] && [ "$_pubs" -gt 0 ] 2>/dev/null; then
        echo
        echo "  ⚠  已經有 $_pubs 個節點在發 /map,而 nav2 的 map_server 還沒啟動 ——"
        echo "     幾乎一定是另一台機器上還開著 SLAM(./run_slam.sh)。它會和 map_server"
        echo "     搶 /map、和 AMCL 搶 map->odom,結果是定位爛掉但完全不報錯。"
        echo "     請先去那台機器把 run_slam.sh 關掉,再回來跑這支。"
        echo '     (ros2 node list | grep slam 可以確認它還在不在)'
        echo
    fi
fi

# 4) Pi 上的鍵盤 teleop。這裡只提醒、不代勞 —— 遠端關別人的東西應該是操作者自己
#    按的。刻意不做自動偵測:那要多一次 ssh + ros2 topic list,每次啟動白等 10 秒,
#    而這行提醒本來就該每次看一眼。
echo
echo "  ⚠  開跑前請確認 Pi 上的 teleop 已關:./robotctl down teleop"
echo "     teleop 閒著時會以 20Hz 持續發零速度到 /cmd_vel(為了餵 driver watchdog),"
echo "     和 nav2 的指令交錯打進來 → 車子抽一下停一下,症狀很像硬體故障。"
echo "     要改回手動駕駛時再 ./robotctl up teleop。"
echo

# --- 發行版 plugin 名稱相容(humble 用 "/",jazzy 之後改成 "::")------------
# Nav2 在 humble → jazzy 之間把 pluginlib 的宣告名稱從 `pkg/ClassName` 改成
# `pkg::ClassName`。config/nav2_params.yaml 是以**實機的 humble 為準**寫的,
# 所以在 jazzy 的模擬機上 planner_server 一 configure 就死:
#
#   FATAL [planner_server]: Failed to create global planner. Exception: ... the
#   class nav2_navfn_planner/NavfnPlanner ... does not exist. Declared types are
#   nav2_navfn_planner::NavfnPlanner ...
#   ERROR [lifecycle_manager_navigation]: Failed to bring up all requested nodes.
#
# 這個症狀很難認:map_server 和 amcl 已經 active(地圖看得到、粒子雲也在),
# 只有 planner 之後的節點是 unconfigured,RViz 只會說「navigate_to_pose action
# server is not available. Is the initial pose set?」—— 把人引去一直重設 2D Pose
# Estimate,但真正的原因和初始位置一點關係也沒有。
#
# 作法是「偵測、然後產生一份改過的暫存參數檔」,而不是改 yaml 本身:比賽用的是
# humble,那份檔案必須維持 humble 正確。偵測方式是問「這台機器上裝的 navfn 到底
# 宣告了哪個名字」,而不是寫死發行版清單 —— 換到再新的發行版也不用再改這裡。
if [[ " $* " != *" params_file:="* ]]; then
    _navfn_share="$(ros2 pkg prefix nav2_navfn_planner 2>/dev/null)/share/nav2_navfn_planner"
    if [ -d "$_navfn_share" ] && ! grep -rqs 'nav2_navfn_planner/NavfnPlanner' "$_navfn_share"; then
        _src_params="$(ros2 pkg prefix car_assemble_description 2>/dev/null)/share/car_assemble_description/config/nav2_params.yaml"
        if [ -f "$_src_params" ]; then
            _patched="${XDG_RUNTIME_DIR:-/tmp}/tirt_nav2_params_${ROS_DISTRO:-unknown}.yaml"
            echo "run_nav2: 偵測到這台的 Nav2 不是 humble 形式,改寫參數檔以求相容"
            echo "          (config/nav2_params.yaml 本身不動,它必須維持實機 humble 正確)。"
            if python3 "$_here/tools/nav2_params_compat.py" "$_src_params" "$_patched"; then
                echo "run_nav2: 相容參數檔 -> $_patched"
                set -- "$@" "params_file:=$_patched"
            else
                echo "run_nav2: 參數改寫失敗,照原檔跑(預期會在 planner_server 掛掉)。" >&2
            fi
        fi
    fi
fi

echo "run_nav2: 使用地圖 $map_file"
echo "run_nav2: RViz 開起來以後 → 車子位置不對就用 '2D Pose Estimate' 校正,"
echo "          然後用 'Nav2 Goal' 在地圖上點目標點並拖出朝向。"
exec ros2 launch car_assemble_description nav2_pc.launch.py "map:=$map_file" "$@"
