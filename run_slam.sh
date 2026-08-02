#!/usr/bin/env bash
# One-click SLAM -- LAPTOP SIDE.
# Runs async_slam_toolbox on the PC (not the Pi): the Pi streams /scan + odom TF over
# DDS, the PC does the heavy scan matching and publishes map->odom + /map back.
# This offloads the CPU-bound scan matching that the Pi 4 couldn't keep up with
# (queue-full scan drops -> rotational map smear). See car_assemble_description/launch/
# slam_pc.launch.py for the full rationale.
#
# Prereqs on the PC:
#   sudo apt install ros-humble-slam-toolbox
#   colcon build --packages-select car_assemble_description   # to get config/ + this launch
#
# 平常不用直接跑這支:./gcs.sh 會在它的 `slam` 視窗裡呼叫。單獨跑也完全沒問題。
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

# 參數會直接交給 ros2 launch,旗標會變成看不懂的 ros2 usage dump。先擋掉。
for _a in "$@"; do
    case "$_a" in
        -h|--help)
            # 用法直接取自檔頭註解(從第 2 行起,遇到第一行非註解就停),不必兩處維護。
            awk 'NR>1 { if (/^#/) { sub(/^# ?/, ""); print } else { exit } }' "$0"; exit 0 ;;
        --nav)
            echo "run_slam: 「--nav」是筆電端 ./gcs.sh 的旗標。這支是「建圖」,和導航是二選一。" >&2
            echo "          要跑導航:./gcs.sh --down 然後 ./gcs.sh --nav(或單獨跑 ./run_nav2.sh)" >&2
            exit 1 ;;
        --no-rviz)
            echo "run_slam: 這支的關 RViz 寫法是 launch 參數:./run_slam.sh use_rviz:=false" >&2
            echo "          (--no-rviz 是 ./gcs.sh 的旗標)" >&2
            exit 1 ;;
        -*)
            echo "run_slam: 不認識的旗標「$_a」。這支只吃 slam_pc.launch.py 的 k:=v 參數," >&2
            echo "          例如 use_rviz:=false、slam_params_file:=/路徑/xxx.yaml。" >&2
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

# TIRT_SIM=1 由 sim/gcs_sim.sh 設定 —— 模擬是單機,兩個節點在同一台電腦上,
# 不需要 Discovery Server、固定別名 IP、interfaceWhiteList。實機那一整套是
# 為了「Pi 和筆電跨 WiFi」而存在的。這個逃生門讓模擬可以直接跑這支腳本本身,
# 而不是另外複製一份 —— 你在模擬裡練的操作,比賽當天是同一個指令。
if [ "${TIRT_SIM:-0}" = 1 ]; then
    echo "run_slam: TIRT_SIM=1 -> 單機模擬模式,略過 DDS 設定。"
else
    # DDS 角色由本機的固定別名 IP 自動判斷,不再需要 DDS_SERVER=<pi_ip>。
    # 別名沒掛上時 setup_dds.sh 會報錯且不設定任何東西 —— 那就別硬跑,不然只會得到
    # 一個永遠等不到 /scan 的 slam_toolbox(看起來活著,實際上什麼都收不到)。
    source "$_here/dds/setup_dds.sh"
    if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
        echo "run_slam: DDS 沒設定好(見上面訊息),中止。" >&2
        exit 1
    fi
fi

# Guard: kill a stale slam from a previous run so map->odom isn't published twice.
pkill -f "async_slam_toolbox_node" 2>/dev/null || true
sleep 1

# slam_toolbox 從 Jazzy(2.8.x)起改成 **lifecycle 節點**,而且不會自己啟動:
# 節點起來了、`ros2 node list` 看得到、log 印完 "Node using stack size" 就沒下文,
# 但它停在 unconfigured,因此**完全沒有訂閱 /scan、也不發 /map**。
# 現象是 RViz 一片空白、`ros2 topic info /map` 的 publisher count 是 0 ——
# 看起來像模擬器或光達壞了,其實只是少了兩個 lifecycle transition。
# (2026-08-01 在 jazzy 開發機上踩到;humble 的 slam_toolbox 是普通節點,沒這問題。)
#
# 這段對兩種版本都安全:humble 上 `ros2 lifecycle get` 會失敗,迴圈空轉後安靜結束。
(
    for _ in $(seq 1 30); do
        sleep 1
        _state="$(ros2 lifecycle get /slam_toolbox 2>/dev/null)" || continue
        case "$_state" in
            active*) exit 0 ;;
            unconfigured*)
                ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1 || exit 0
                ros2 lifecycle set /slam_toolbox activate  >/dev/null 2>&1 || exit 0
                echo "run_slam: slam_toolbox 是 lifecycle 節點(jazzy+),已自動 configure + activate。"
                exit 0 ;;
        esac
    done
) &

exec ros2 launch car_assemble_description slam_pc.launch.py "$@"
