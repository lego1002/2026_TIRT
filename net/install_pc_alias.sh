#!/usr/bin/env bash
# 在「筆電」上掛一個永久的固定別名 IP(預設 10.77.0.1/24),對應 Pi 的 10.77.0.2。
#
#   sudo ./net/install_pc_alias.sh              # 自動挑無線介面
#   sudo TIRT_IFACE=wlp3s0 ./net/install_pc_alias.sh
#
# 為什麼不直接寫進某個 wifi 連線設定:那樣每加一個新場地的 SSID 就要重設一次。
# 這裡改成掛在「介面」層級,連到任何網路都會有,才符合「換場地零設定」。
#
# 會依筆電實際用的網路管理程式擇一安裝:
#   NetworkManager  -> /etc/NetworkManager/dispatcher.d/90-tirt-alias(每次連線事件補上)
#   systemd-networkd -> /etc/systemd/network/<iface>.network.d/tirt-alias.conf
# 兩者都會在重開機 / 換 SSID / wifi 重連後自動把別名補回來。
set -euo pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
source "$_here/tirt_net.conf"

ALIAS="${TIRT_PC_IP}/${TIRT_PREFIX}"

if [ "$(id -u)" -ne 0 ]; then
    echo "請用 sudo 執行:sudo $0" >&2
    exit 1
fi

# --- 挑介面 ---------------------------------------------------------------
if [ -n "${TIRT_IFACE:-}" ]; then
    IFACE="$TIRT_IFACE"
else
    # 第一張無線網卡;沒有的話退回第一張有 global IPv4 且不是虛擬/VPN 的介面。
    IFACE="$(ls /sys/class/net/*/wireless 2>/dev/null | head -1 | cut -d/ -f5 || true)"
    if [ -z "$IFACE" ]; then
        IFACE="$(ip -4 -o addr show scope global \
            | awk '$2 !~ /^(tailscale|lo|docker|veth|br-|virbr|zt|wg)/ {print $2; exit}')"
    fi
fi
if [ -z "$IFACE" ] || [ ! -d "/sys/class/net/$IFACE" ]; then
    echo "找不到可用介面。用 TIRT_IFACE=<iface> 指定,例如 sudo TIRT_IFACE=wlp3s0 $0" >&2
    exit 1
fi
echo "介面: $IFACE   別名: $ALIAS"

# --- 立刻生效(不必等重連)-------------------------------------------------
if ip -4 -o addr show dev "$IFACE" | grep -q "${TIRT_PC_IP}/"; then
    echo "別名已存在,略過即時新增。"
else
    ip addr add "$ALIAS" dev "$IFACE"
    echo "已即時加上 $ALIAS。"
fi

# --- 持久化 ---------------------------------------------------------------
if systemctl is-active --quiet NetworkManager && nmcli -t dev status 2>/dev/null \
        | grep -q "^${IFACE}:.*:\(connected\|disconnected\):"; then
    DISPATCH=/etc/NetworkManager/dispatcher.d/90-tirt-alias
    cat > "$DISPATCH" <<EOF
#!/bin/sh
# TIRT: 每次 $IFACE 連上網路後補回固定別名 $ALIAS。
# 由 2026_TIRT/net/install_pc_alias.sh 產生 —— 不要手改,重跑那支腳本即可。
IFACE="\$1"
ACTION="\$2"
[ "\$IFACE" = "$IFACE" ] || exit 0
case "\$ACTION" in
    up|dhcp4-change|connectivity-change) ;;
    *) exit 0 ;;
esac
ip -4 -o addr show dev "\$IFACE" | grep -q "${TIRT_PC_IP}/" && exit 0
ip addr add $ALIAS dev "\$IFACE" || true
EOF
    chmod 755 "$DISPATCH"
    chown root:root "$DISPATCH"
    echo "已安裝 NetworkManager dispatcher: $DISPATCH"
elif systemctl is-active --quiet systemd-networkd; then
    # 找出這張介面實際套用的 .network 檔,對它加 drop-in。
    NETFILE="$(networkctl status "$IFACE" 2>/dev/null \
        | awk -F': *' '/Network File/ {print $2; exit}')"
    if [ -z "$NETFILE" ] || [ "$NETFILE" = "n/a" ]; then
        echo "無法判斷 $IFACE 的 .network 檔;請手動加 Address=$ALIAS。" >&2
        exit 1
    fi
    DROPDIR="/etc/systemd/network/$(basename "$NETFILE").d"
    mkdir -p "$DROPDIR"
    cat > "$DROPDIR/tirt-alias.conf" <<EOF
# TIRT: 固定別名,由 2026_TIRT/net/install_pc_alias.sh 產生。
[Network]
Address=$ALIAS
EOF
    networkctl reload 2>/dev/null || systemctl restart systemd-networkd
    echo "已安裝 networkd drop-in: $DROPDIR/tirt-alias.conf"
else
    echo "偵測不到 NetworkManager 或 systemd-networkd —— 別名只有這次生效," >&2
    echo "重開機後會消失。請自行加入你的網路管理設定。" >&2
fi

echo
echo "=== 驗證 ==="
ip -4 -o addr show dev "$IFACE" | sed 's/^/  /'
echo "接著測:ping -c3 ${TIRT_PI_IP}   (Pi 那邊也要先跑過 install_pi_network.sh)"
