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
set -e

DS_PORT="${DDS_SERVER_PORT:-11811}"

source /opt/ros/humble/setup.bash

# One server only -- a second instance on the same port would fight for it.
pkill -f "fastdds discovery|fast-discovery-server" 2>/dev/null || true
sleep 0.5

# Bind to all interfaces (default when -l is omitted). Clients reach it at the Pi's
# fixed alias 10.77.0.2, which is hard-coded in both fastdds_*.xml profiles; binding
# to 0.0.0.0 also keeps it reachable over the DHCP address for ad-hoc debugging.
echo "Starting Fast DDS Discovery Server: server-id 0, port ${DS_PORT}"
exec fastdds discovery -i 0 -p "${DS_PORT}"
