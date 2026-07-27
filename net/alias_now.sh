#!/usr/bin/env bash
# 立刻掛上固定別名 IP —— **不持久,重開機 / 重連 WiFi 就沒了**。
#
#   sudo ./net/alias_now.sh pi     # 樹莓派 → 10.77.0.2
#   sudo ./net/alias_now.sh pc     # 筆電   → 10.77.0.1
#   sudo ./net/alias_now.sh pi --off   # 拿掉
#
# 這是「今天想馬上跑,還沒空做持久設定」用的捷徑。正式做法:
#   Pi   → sudo ./net/install_pi_network.sh(順便把各場地 WiFi 寫進 netplan)
#   筆電 → sudo ./net/install_pc_alias.sh (裝 dispatcher,每次連線自動補回來)
# 兩者做完之後就不需要再跑這支了。
set -euo pipefail

_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
source "$_here/tirt_net.conf"

ROLE="${1:-}"
OFF=0
[ "${2:-}" = "--off" ] && OFF=1

case "$ROLE" in
    pi) IP="$TIRT_PI_IP" ;;
    pc) IP="$TIRT_PC_IP" ;;
    *)  echo "用法: sudo $0 pi|pc [--off]" >&2; exit 1 ;;
esac

[ "$(id -u)" -eq 0 ] || { echo "請用 sudo 執行" >&2; exit 1; }

# 挑無線介面(和 install_pc_alias.sh 同一套邏輯);可用 TIRT_IFACE 指定。
IFACE="${TIRT_IFACE:-}"
if [ -z "$IFACE" ]; then
    IFACE="$(ls /sys/class/net/*/wireless 2>/dev/null | head -1 | cut -d/ -f5 || true)"
fi
if [ -z "$IFACE" ]; then
    IFACE="$(ip -4 -o addr show scope global \
        | awk '$2 !~ /^(tailscale|lo|docker|veth|br-|virbr|zt|wg)/ {print $2; exit}')"
fi
[ -n "$IFACE" ] || { echo "找不到介面,用 TIRT_IFACE=<iface> 指定" >&2; exit 1; }

if [ "$OFF" = 1 ]; then
    ip addr del "${IP}/${TIRT_PREFIX}" dev "$IFACE" 2>/dev/null \
        && echo "已移除 ${IP} (${IFACE})" || echo "${IP} 本來就不在 ${IFACE} 上"
    exit 0
fi

if ip -4 -o addr show dev "$IFACE" | grep -q " ${IP}/"; then
    echo "${IP} 已經在 ${IFACE} 上了。"
else
    ip addr add "${IP}/${TIRT_PREFIX}" dev "$IFACE"
    echo "已加上 ${IP}/${TIRT_PREFIX} 到 ${IFACE}(暫時性,重開機會消失)"
fi
ip -4 -o addr show dev "$IFACE" | sed 's/^/  /'
