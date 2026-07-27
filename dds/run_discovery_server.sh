#!/usr/bin/env bash
# Fast DDS Discovery Server -- runs on the Pi (server-id 0, port 11811).
#
# Why this exists: this venue's WiFi AP does not forward multicast between wireless
# clients, so DDS's default multicast discovery (SPDP) never connects the Pi and PC
# (ping works, `ros2 multicast send/receive` does not). This server is a fixed
# unicast rendezvous point -- every ROS node on both machines is a SUPER_CLIENT of
# it via dds/setup_dds.sh, which points them at the Pi's fixed alias IP 10.77.0.2.
#
# Normally you don't run this by hand: pi/robot_tmux.sh gives it its own tmux window
# (`dds`) so its output is visible, and `./robotctl up` on the laptop starts that.
# Leave server-id 0 -- its GUID prefix 44.53.00.5f... is hard-coded as the
# RemoteServer prefix in dds/fastdds_pi.xml and dds/fastdds_pc.xml.
#
#   ./dds/run_discovery_server.sh          # foreground, Ctrl+C to stop
# Deliberately no `set -e`: sourcing ROS's setup.bash can return non-zero on a
# harmless internal step, and under `set -e` that aborts the script before it ever
# starts the server -- which looks exactly like "the dds window died on its own"
# with no message. Failures below are checked explicitly instead.

DS_PORT="${DDS_SERVER_PORT:-11811}"

# server 的設定全部來自下面的 CLI 參數(-i / -p,綁 0.0.0.0),不需要 client 用的 profile。
# 清掉它讓這件事明確,也避免日後改 profile 時意外影響到 server。
# ※ 2026-07-27 實測過:即使**不**清掉,帶著 fastdds_pi.xml 跑 server 仍然正常
#   (Participant Type 照樣是 SERVER)。曾一度誤判成「profile 會讓 server 卡死」,
#   那是把輸出接到 `| head` 造成的緩衝假象,不是真的。所以這行是保險,不是修 bug。
unset FASTRTPS_DEFAULT_PROFILES_FILE

source /opt/ros/humble/setup.bash

if ! command -v fastdds >/dev/null 2>&1; then
    echo "run_discovery_server: 找不到 fastdds 指令 —— ROS 沒 source 到?" >&2
    echo "  試: source /opt/ros/humble/setup.bash && which fastdds" >&2
    exit 1
fi

# One server only -- a second instance on the same port would fight for it.
pkill -f "fastdds discovery|fast-discovery-server" 2>/dev/null || true
sleep 0.5

# The server is the rendezvous point for BOTH machines; if it exits, everything
# silently stops discovering each other. Print why instead of just vanishing.
trap 'echo "run_discovery_server: 結束(離開碼 $?)。上面若有錯誤訊息就是原因。" >&2' EXIT

# Bind to all interfaces (default when -l is omitted). Clients reach it at the Pi's
# fixed alias 10.77.0.2, which is hard-coded in both fastdds_*.xml profiles; binding
# to 0.0.0.0 also keeps it reachable over the DHCP address for ad-hoc debugging.
echo "Starting Fast DDS Discovery Server: server-id 0, port ${DS_PORT}"
exec fastdds discovery -i 0 -p "${DS_PORT}"
