#!/usr/bin/env bash
# Fast DDS Discovery Server -- runs on the Pi (server-id 0, port 11811).
#
# Why this exists: this venue's WiFi AP does not forward multicast between wireless
# clients, so DDS's default multicast discovery (SPDP) never connects the Pi and PC
# (ping works, `ros2 multicast send/receive` does not). This server is a fixed
# unicast rendezvous point -- every ROS node on both machines is configured as a
# CLIENT of it via dds/setup_dds.sh (which renders @SERVER_IP@ into the profile).
#
# run_robot.sh starts this automatically in the background; run it by hand only to
# host the server without the full bringup. Leave server-id 0 (its GUID prefix
# 44.53.00.5f... is hard-coded as the RemoteServer prefix in fastdds_lan.xml).
#
#   ./dds/run_discovery_server.sh          # foreground, Ctrl+C to stop
set -e

DS_PORT="${DDS_SERVER_PORT:-11811}"

source /opt/ros/humble/setup.bash

# One server only -- a second instance on the same port would fight for it.
pkill -f "fastdds discovery|fast-discovery-server" 2>/dev/null || true
sleep 0.5

# Bind to all interfaces (default when -l is omitted): robust to the Pi's LAN IP
# changing on DHCP. Clients still reach it at the Pi's current LAN IP, which
# setup_dds.sh writes into their profile as @SERVER_IP@.
echo "Starting Fast DDS Discovery Server: server-id 0, port ${DS_PORT}"
exec fastdds discovery -i 0 -p "${DS_PORT}"
