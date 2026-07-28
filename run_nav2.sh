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

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi

# DDS 角色由本機的固定別名 IP 自動判斷。別名沒掛上時 setup_dds.sh 會報錯且不設定
# 任何東西 —— 那就別硬跑,不然只會得到一整包永遠收不到 /scan 的 nav2 節點
# (lifecycle 會 active,看起來很健康,實際上什麼都收不到)。
source "$_here/dds/setup_dds.sh"
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    echo "run_nav2: DDS 沒設定好(見上面訊息),中止。" >&2
    exit 1
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

if [ ! -f "$map_file" ]; then
    echo "run_nav2: 找不到地圖 $map_file" >&2
    echo "現有的地圖:" >&2
    ls -1 "$_here/maps/"*.yaml 2>/dev/null | xargs -r -n1 basename >&2
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

# 3) Pi 上的鍵盤 teleop。這裡只提醒、不代勞 —— 遠端關別人的東西應該是操作者自己
#    按的。刻意不做自動偵測:那要多一次 ssh + ros2 topic list,每次啟動白等 10 秒,
#    而這行提醒本來就該每次看一眼。
echo
echo "  ⚠  開跑前請確認 Pi 上的 teleop 已關:./robotctl down teleop"
echo "     teleop 閒著時會以 20Hz 持續發零速度到 /cmd_vel(為了餵 driver watchdog),"
echo "     和 nav2 的指令交錯打進來 → 車子抽一下停一下,症狀很像硬體故障。"
echo "     要改回手動駕駛時再 ./robotctl up teleop。"
echo

echo "run_nav2: 使用地圖 $map_file"
echo "run_nav2: RViz 開起來以後 → 車子位置不對就用 '2D Pose Estimate' 校正,"
echo "          然後用 'Nav2 Goal' 在地圖上點目標點並拖出朝向。"
exec ros2 launch car_assemble_description nav2_pc.launch.py "map:=$map_file" "$@"
