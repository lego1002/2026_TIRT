#!/usr/bin/env bash
# 在「樹莓派」上安裝 TIRT 無線網路設定(多 SSID + 固定第二 IP)。
#
#   1. cp net/60-tirt-wifi.yaml.example net/60-tirt-wifi.yaml
#   2. 編輯 net/60-tirt-wifi.yaml,填入手機熱點 SSID/密碼與 RMML_2G 密碼
#   3. sudo ./net/install_pi_network.sh
#
# 安全性:用 `netplan try` 套用 —— 它會等你按 Enter 確認,120 秒內沒確認就自動
# 回滾成原本的設定。萬一新設定讓 wifi 連不上(SSH 當場斷線),你什麼都不用做,
# 兩分鐘後 Pi 會自己回到現在能用的狀態。**絕對不要改用 netplan apply。**
set -euo pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
source "$_here/tirt_net.conf"

SRC="$_here/60-tirt-wifi.yaml"
DST="/etc/netplan/60-tirt-wifi.yaml"

if [ "$(id -u)" -ne 0 ]; then
    echo "請用 sudo 執行:sudo $0" >&2
    exit 1
fi

if [ ! -f "$SRC" ]; then
    echo "找不到 $SRC" >&2
    echo "先做:cp $_here/60-tirt-wifi.yaml.example $SRC 然後填入 SSID/密碼。" >&2
    exit 1
fi

if grep -qE 'PHONE_HOTSPOT_(SSID|PASSWORD)|RMML_2G_PASSWORD' "$SRC"; then
    echo "$SRC 裡還留著樣板佔位字串,請先填入真正的 SSID / 密碼。" >&2
    exit 1
fi

echo "=== 目前 /etc/netplan/ 內容(套用前先看一眼,確認不會互相打架)==="
for f in /etc/netplan/*.yaml; do
    [ -e "$f" ] || continue
    echo "--- $f ---"
    cat "$f"
done
echo

# 備份既有的同名檔(重跑本腳本時)
if [ -f "$DST" ]; then
    cp -a "$DST" "${DST}.bak.$(date +%s)"
    echo "已備份舊的 $DST"
fi

install -m 600 -o root -g root "$SRC" "$DST"
echo "已安裝 $DST"

# 語法檢查(不套用)。有錯就直接停,不要冒險 try。
netplan generate

echo
echo "=== 即將 netplan try ==="
echo "如果 SSH 斷了不要慌:120 秒後會自動回滾,重連即可。"
echo "連線正常的話,在提示出現時按 Enter 確認。"
echo
netplan try

echo
echo "=== 驗證 ==="
ip -4 -o addr show dev wlan0 | sed 's/^/  /'
if ip -4 -o addr show dev wlan0 | grep -q "${TIRT_PI_IP}/"; then
    echo "OK: 固定別名 ${TIRT_PI_IP} 已就緒。"
else
    echo "警告: 沒看到 ${TIRT_PI_IP} —— 檢查 60-tirt-wifi.yaml 的 addresses 欄位。" >&2
    exit 1
fi
echo "接著在筆電上跑一次:sudo ./net/install_pc_alias.sh,然後互 ping 驗證。"
