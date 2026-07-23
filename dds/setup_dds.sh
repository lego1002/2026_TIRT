# Render the Fast DDS LAN profile for THIS machine and export
# FASTRTPS_DEFAULT_PROFILES_FILE. SOURCE it (don't execute) so the env var lands
# in your shell -- run it once per machine, and again after changing venue/network:
#
#     # Pi (also the discovery-server host): server defaults to its own LAN IP
#     source /home/lego/2026_TIRT/dds/setup_dds.sh
#     # PC: point at where the discovery server runs (= the Pi's LAN IP)
#     DDS_SERVER=192.168.0.70 source /home/lego/2026_TIRT/dds/setup_dds.sh
#
# The rendered profile does TWO things (see dds/fastdds_lan.xml for the full why):
#   1. discovery via a Fast DDS Discovery Server (unicast) -- this venue's WiFi AP
#      does not forward multicast between clients, so DDS's default multicast SPDP
#      never connects the two machines. The server (dds/run_discovery_server.sh,
#      auto-started by run_robot.sh on the Pi) gives a fixed unicast rendezvous.
#   2. interfaceWhiteList so bulk data (/robot_description, /tf, /map) stays on the
#      LAN interface and never fragments over tailscale's 1280-MTU link.
#
# Env overrides:
#   DDS_IFACE=eth0      force the LAN interface (auto-detect skips tailscale/lo/docker/virtual)
#   DDS_SERVER=<ip>     discovery-server IP (default: this machine's own LAN IP, correct on the Pi)
#   DDS_SERVER_PORT=n   discovery-server port (default 11811)

_dds_dir="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"
_dds_template="$_dds_dir/fastdds_lan.xml"

if [ -n "$DDS_IFACE" ]; then
    _dds_iface="$DDS_IFACE"
else
    # First UP global IPv4 interface that isn't a VPN/loopback/virtual device.
    _dds_iface="$(ip -4 -o addr show scope global 2>/dev/null \
        | awk '$2 !~ /^(tailscale|lo|docker|veth|br-|virbr|zt|wg)/ {print $2; exit}')"
fi

if [ -z "$_dds_iface" ]; then
    echo "setup_dds: no LAN interface found -- set DDS_IFACE=<iface> and re-source." >&2
elif ! _dds_ip="$(ip -4 -o addr show dev "$_dds_iface" scope global 2>/dev/null \
        | awk '{sub(/\/.*/, "", $4); print $4; exit}')" || [ -z "$_dds_ip" ]; then
    echo "setup_dds: interface '$_dds_iface' has no IPv4 address." >&2
elif [ ! -r "$_dds_template" ]; then
    echo "setup_dds: template not found: $_dds_template" >&2
else
    # Discovery-server address: default to THIS machine's LAN IP. On the Pi (which
    # hosts the server) that is correct with no extra config; on the PC you must
    # pass DDS_SERVER=<pi_ip> or it will point the client at itself and never connect.
    _dds_server="${DDS_SERVER:-$_dds_ip}"
    _dds_server_port="${DDS_SERVER_PORT:-11811}"

    _dds_out="${XDG_RUNTIME_DIR:-/tmp}/fastdds_active.xml"
    if sed -e "s/@LAN_IP@/$_dds_ip/g" \
           -e "s/@SERVER_IP@/$_dds_server/g" \
           -e "s/@SERVER_PORT@/$_dds_server_port/g" \
           "$_dds_template" > "$_dds_out"; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="$_dds_out"
        echo "setup_dds: iface=$_dds_iface ip=$_dds_ip  server=$_dds_server:$_dds_server_port -> $_dds_out"
        if [ "$_dds_server" = "$_dds_ip" ]; then
            echo "setup_dds: (server = this machine -- correct on the Pi; on the PC set DDS_SERVER=<pi_ip>)"
        fi
        # The ros2 daemon caches discovery config from whenever it first started, so
        # a daemon spawned before this profile existed will ignore it ("topic list
        # shows nothing / echo says no type"). Clear it so the next ros2 command
        # respawns the daemon with this profile.
        if command -v ros2 >/dev/null 2>&1; then ros2 daemon stop >/dev/null 2>&1 || true; fi
    else
        echo "setup_dds: failed to render $_dds_out" >&2
    fi
fi

unset _dds_dir _dds_template _dds_iface _dds_ip _dds_server _dds_server_port _dds_out
