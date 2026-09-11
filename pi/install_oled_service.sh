#!/usr/bin/env bash
# 在「樹莓派」上安裝 / 啟用開機自動點亮 OLED 的 systemd 服務。
#
#   sudo ./pi/install_oled_service.sh            # 安裝 + 立刻啟動 + 開機自動起
#   sudo ./pi/install_oled_service.sh --uninstall
#
# 之後的日常操作(不必再跑這支):
#   systemctl status tirt-oled          # 現在的狀態
#   journalctl -u tirt-oled -f          # 即時輸出
#   sudo systemctl restart tirt-oled    # 改過 oled_status_node.py 並 colcon build 之後
#   sudo systemctl stop tirt-oled       # 要用 tools/oled_test.py 或 bringup 的 use_oled 之前
#
# 為什麼這一段是 systemd,而 bringup 卻刻意用 tmux(pi/robot_tmux.sh 開頭有解釋):
#   兩者的需求正好相反。bringup 是「要看得見、要能單段重跑」的除錯對象;OLED 狀態顯示是
#   基礎設施 —— 它必須在沒有人、沒有筆電、沒有 ssh 的情況下自己活著,而且掛了要自己重來。
#   那正是 systemd 的工作(Restart=always)。
#
# unit 檔用 heredoc 就地產生而不是放一份 .service 進 repo:裡面的路徑和使用者是「這台機器
# 的」,追蹤一份寫死 /home/lego 的檔案只會多一個會過期的真相來源。
set -euo pipefail

UNIT=/etc/systemd/system/tirt-oled.service
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
REPO="$( cd "$_here/.." && pwd )"

[ "$(id -u)" = 0 ] || { echo "install_oled_service: 請用 sudo 跑這支。" >&2; exit 1; }

# 服務要以「平常在用的那個帳號」執行,不能用 root:luma 的 python 套件是 pip --user 裝的
# (在 ~/.local),而且 /dev/spidev0.0 與 /dev/gpiomem 是靠 dialout 群組拿到的權限。
RUN_USER="${SUDO_USER:-lego}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
[ -n "$RUN_HOME" ] || { echo "install_oled_service: 找不到使用者 $RUN_USER 的家目錄。" >&2; exit 1; }

if [ "${1:-}" = "--uninstall" ]; then
    systemctl disable --now tirt-oled.service 2>/dev/null || true
    rm -f "$UNIT"
    systemctl daemon-reload
    echo "install_oled_service: 已移除 tirt-oled.service。"
    exit 0
fi

# 同時有兩個行程畫同一片 SPI 面板 = 花屏(SPI 沒有互斥鎖)。bringup 的 use_oled 預設是
# false,但參數檔是持久的,被人打開過就會一直留著,所以在這裡先講清楚。
ARGS_FILE="$RUN_HOME/.tirt_robot_args"
if grep -q "use_oled:=true" "$ARGS_FILE" 2>/dev/null; then
    echo "install_oled_service: 注意 —— $ARGS_FILE 裡有 use_oled:=true。" >&2
    echo "                      bringup 會再開一個 oled_status 跟這個服務搶同一片面板(花屏)。" >&2
    echo "                      請拿掉它:./robotctl args \"...\"(不含 use_oled),或 Pi 上直接編輯該檔。" >&2
fi

cat > "$UNIT" <<UNITEOF
# 由 $REPO/pi/install_oled_service.sh 產生,請不要手改這裡 —— 改 pi/oled_boot.sh。
[Unit]
Description=TIRT 車上 SPI OLED 狀態顯示
# 等網路就緒只是為了讓第一張畫面就有正確的 IP;等不到也照跑(oled_boot.sh 自己有
# 45 秒的別名等待 + 降級模式),因為「沒有網路」正是最需要看螢幕的那一刻。
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$REPO
Environment=HOME=$RUN_HOME
EnvironmentFile=-/etc/default/tirt-oled
ExecStart=$REPO/pi/oled_boot.sh
Restart=always
RestartSec=5
# 一定要 SIGINT:節點只對 SIGINT 做乾淨收尾(送出 "ROS stopped" 那張畫面再退出);
# 預設的 SIGTERM 會讓它死在半路,面板停在最後一張正常畫面上,看起來像還活著。
KillSignal=SIGINT
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
UNITEOF

systemctl daemon-reload
systemctl enable --now tirt-oled.service
echo "install_oled_service: 已安裝並啟動。狀態:"
systemctl --no-pager --lines=0 status tirt-oled.service || true
echo
echo "看即時輸出:journalctl -u tirt-oled -f"
