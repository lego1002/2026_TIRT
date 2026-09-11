#!/usr/bin/env bash
# 開機自動點亮車上 OLED —— 由 systemd 服務 tirt-oled.service 呼叫(見 pi/install_oled_service.sh)。
#
# 為什麼 OLED 要獨立於 bringup 之外自己開機起來:
#   robot_bringup.launch.py 的 use_oled 只有在「有人跑 bringup」時才會點亮螢幕,而 bringup
#   是筆電端 ./gcs.sh / ./robotctl 手動拉起來的。於是重開機後(還沒開筆電、或比賽當天根本
#   沒有筆電)螢幕是全黑的 —— 而黑畫面跟「Pi 沒開機」「OLED 壞了」看起來一模一樣。
#   這支讓螢幕在**開機就亮**,即使 ROS 其他節點都還沒起來:
#     - 第一行的 IP 是唯一「不可能從 PC 端檢查」的檢查項(開機沒拿到 10.77.0.2 的機器人,
#       正好就是 PC 看不到的那台機器人),沒有螢幕就只能瞎猜。
#     - 電量 / scan / odom 速率則會在 bringup 起來的當下自己開始跳動,不必重開這支。
#
# 所以請維持 robot_bringup.launch.py 的 use_oled 預設 false:SPI 沒有互斥鎖,兩個行程
# 同時畫同一片面板只會得到花屏。要用 bringup 那份就先 `sudo systemctl stop tirt-oled`。
# 不用 set -u:/opt/ros/humble/setup.bash 會讀未設定的 AMENT_TRACE_SETUP_FILES,開了就當場死在第一行。
set -o pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
REPO="$( cd "$_here/.." && pwd )"
source "$REPO/net/tirt_net.conf"

# 開機時 wifi 要幾秒才會關聯上、networkd 才會掛好 10.77.0.2 別名。
ALIAS_WAIT=${TIRT_OLED_ALIAS_WAIT:-45}

source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"

has_alias() { ip -4 -o addr show scope global 2>/dev/null | grep -q " ${TIRT_PI_IP}/"; }

for _i in $(seq 1 "$ALIAS_WAIT"); do
    has_alias && break
    sleep 1
done

# 有別名才套 DDS profile。setup_dds.sh 在沒有別名時會刻意什麼都不設定,這裡照它的判斷走。
if has_alias; then
    source "$REPO/dds/setup_dds.sh"
fi

if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    # 降級模式:沒有固定別名(常見於「沒帶到 wifi 的純電池開機」)。
    # 這時**照樣把螢幕點亮** —— 這正是最需要螢幕的一刻,面板上會顯示當下的 DHCP 位址
    # (或 no-net),操作者一眼就知道為什麼筆電連不到。
    # 只是把它關在 localhost:沒有 profile 就硬連 discovery server 沒有意義,而且會讓
    # 它用一個未經白名單約束的路徑亂發現(tailscale/docker0),那是我們一路在避免的事。
    echo "oled_boot: 沒有固定別名 ${TIRT_PI_IP},以 localhost 模式點亮面板(只顯示 IP,收不到 ROS 主題)。" >&2
    export ROS_LOCALHOST_ONLY=1
    # 別名之後才出現的話(例如稍後才連上 wifi),送 SIGINT 讓自己收工,交給 systemd
    # Restart=always 重新起一次 —— 那一次就會走到上面的完整模式。
    _self=$$
    ( while ! has_alias; do sleep 5; done
      echo "oled_boot: ${TIRT_PI_IP} 出現了,重啟 oled_status 以套用 DDS profile。" >&2
      kill -INT "$_self" ) &
fi

exec ros2 run ominibot_driver oled_status --ros-args \
    -p controller:="${TIRT_OLED_CONTROLLER:-ssd1306}" \
    -p gpio_dc:="${TIRT_OLED_DC:-24}" \
    -p gpio_rst:="${TIRT_OLED_RST:-25}" \
    -p batt_full_v:="${TIRT_OLED_BATT_FULL:-12.6}" \
    -p batt_empty_v:="${TIRT_OLED_BATT_EMPTY:-10.5}"
