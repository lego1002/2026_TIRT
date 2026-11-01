#!/usr/bin/env bash
# 在「樹莓派」上安裝網路線直連的固定位址(eth0 = 10.78.0.2)。
#
#   sudo ./net/install_pi_eth.sh
#
# 只動 eth0,不碰 wlan0。仍然用 `netplan try`(120 秒沒按 Enter 自動回滾),
# 理由同 install_pi_network.sh:netplan 會重套「所有」設定,包括 wifi。
set -euo pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
source "$_here/tirt_net.conf"

SRC="$_here/70-tirt-eth.yaml"
DST="/etc/netplan/70-tirt-eth.yaml"

if [ "$(id -u)" -ne 0 ]; then
    echo "請用 sudo 執行:sudo $0" >&2
    exit 1
fi

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
ip -4 -o addr show dev eth0 | sed 's/^/  /'
if ip -4 -o addr show dev eth0 | grep -q "${TIRT_PI_ETH_IP}/"; then
    echo "OK: 網路線直連位址 ${TIRT_PI_ETH_IP} 已就緒。"
    echo "筆電那張網卡設 ${TIRT_PC_ETH_IP} / 255.255.255.0(不填閘道),然後 ssh ${TIRT_SSH_USER}@${TIRT_PI_ETH_IP}"
else
    echo "警告: eth0 上沒看到 ${TIRT_PI_ETH_IP}。" >&2
    exit 1
fi
