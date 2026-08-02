#!/usr/bin/env bash
# 模擬環境一次性安裝 —— 在**任何一台**電腦上把模擬跑起來需要的全部準備。
#
#   ./sim/setup_sim.sh          # 檢查 + 建立 workspace + build
#   ./sim/setup_sim.sh --check  # 只檢查,什麼都不做
#
# 它做的事:
#   1. 找出這台機器裝的是哪個 ROS 2 發行版(不寫死 humble —— 開發機可能是 jazzy)
#   2. 檢查缺哪些 apt 套件,把安裝指令印出來(不會自己 sudo)
#   3. 把三個套件 symlink 進 ~/ros2_ws/src 並 colcon build
#
# ※ 為什麼是 ~/ros2_ws 而不是在 repo 裡 build:CLAUDE.md 明令「絕不在 repo 根目錄
#   colcon build」。in-tree 的 install/ 會蓋掉真正的 workspace,而且它是 build 當下的
#   死副本 —— 2026-07-25 就因此白白 debug 了一整場(跑的是舊程式,改的是新的)。
set -uo pipefail
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
_repo="$( cd "$_here/.." && pwd )"
WS="$HOME/ros2_ws"
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

# --- 1. 找 ROS 發行版 -------------------------------------------------------
# repo 的實機腳本寫死 /opt/ros/humble(Pi 和比賽用筆電都是 humble),但開發機
# 不一定。模擬不碰硬體,所以哪個發行版都能跑,這裡自動挑一個。
ROS_D=""
for d in humble jazzy iron rolling kilted; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then ROS_D="$d"; break; fi
done
if [ -z "$ROS_D" ]; then
    echo "找不到任何 ROS 2 安裝(/opt/ros/*/setup.bash)。" >&2
    echo "請先安裝 ROS 2:https://docs.ros.org/en/humble/Installation.html" >&2
    exit 1
fi
echo "ROS 2 發行版: $ROS_D"
if [ "$ROS_D" != humble ]; then
    echo "  ※ 注意:實機(Pi + 比賽筆電)用的是 humble,這台是 $ROS_D。"
    echo "    模擬本身沒差,但 Nav2 的參數檔在不同版本間偶有增刪 —— 若 nav2 啟動時"
    echo "    抱怨某個參數不認得,以實機的 humble 為準,不要改 config/nav2_params.yaml。"
fi

set +u; source "/opt/ros/$ROS_D/setup.bash"; set -u

# --- 2. 檢查 apt 相依 -------------------------------------------------------
need=()
for p in slam-toolbox navigation2 nav2-bringup joint-state-publisher \
         robot-state-publisher rviz2 xacro tf-transformations; do
    ros2 pkg prefix "${p//-/_}" >/dev/null 2>&1 || need+=("ros-$ROS_D-$p")
done
command -v colcon >/dev/null || need+=("python3-colcon-common-extensions")
python3 -c "import yaml, numpy" 2>/dev/null || need+=("python3-yaml python3-numpy")
command -v tmux >/dev/null || need+=("tmux")

if [ ${#need[@]} -gt 0 ]; then
    echo
    echo "缺少套件,請執行(在 Claude Code 裡可以打 '! ' 開頭直接跑):"
    echo
    echo "  sudo apt update && sudo apt install -y ${need[*]}"
    echo
    [ "$CHECK_ONLY" = 1 ] && exit 1
    read -r -p "裝好了嗎?按 Enter 繼續、Ctrl-C 中止 " _ || exit 1
fi

[ "$CHECK_ONLY" = 1 ] && { echo "檢查完成。"; exit 0; }

# --- 3. workspace ----------------------------------------------------------
mkdir -p "$WS/src"
link() {   # link <來源目錄> <src 底下的名字>
    local src="$1" name="$2" dst="$WS/src/$2"
    if [ -L "$dst" ]; then
        [ "$(readlink -f "$dst")" = "$(readlink -f "$src")" ] && return 0
        echo "  ! $name 已連到別的地方($(readlink -f "$dst")),跳過" >&2; return 0
    fi
    [ -e "$dst" ] && { echo "  ! $name 已存在且不是 symlink,跳過" >&2; return 0; }
    ln -s "$src" "$dst" && echo "  + $name -> $src"
}
echo
echo "建立 workspace symlink($WS/src):"
link "$_repo/car_assemble_description" car_assemble_description
link "$_repo/ominibot_driver" ominibot_driver
link "$_here" tirt_sim

echo
echo "colcon build..."
# --symlink-install:改 launch/config/worlds 不必重 build,只要重啟節點。
# ament_python 套件的 .py 仍然要重 build 才會生效(這點和實機一樣)。
( cd "$WS" && colcon build --symlink-install \
    --packages-select car_assemble_description ominibot_driver tirt_sim ) || exit 1

echo
echo "======================================================================"
echo " 完成。接下來:"
echo
echo "   source $WS/install/setup.bash"
echo "   ./sim/gcs_sim.sh            # 建圖模式(對應實機的 ./gcs.sh)"
echo "   ./sim/gcs_sim.sh --nav      # 導航模式(對應實機的 ./gcs.sh --nav)"
echo
echo " 第一次請先讀 sim/README.md 的「學習路線」。"
echo "======================================================================"
