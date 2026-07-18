# Render the Fast DDS LAN profile with THIS machine's current LAN IP and export
# FASTRTPS_DEFAULT_PROFILES_FILE. SOURCE it (don't execute) so the env var lands
# in your shell -- run it once per machine, and again after changing venue/network:
#
#     source /home/lego/2026_TIRT/dds/setup_dds.sh
#
# The LAN interface is auto-detected: the first global-scope IPv4 interface that
# isn't tailscale/loopback/docker/virtual. Override if the guess is wrong:
#
#     DDS_IFACE=eth0 source /home/lego/2026_TIRT/dds/setup_dds.sh
#
# It renders dds/fastdds_lan.xml (a template with an @LAN_IP@ placeholder) to a
# runtime file and points FASTRTPS_DEFAULT_PROFILES_FILE at it. Do NOT point that
# env var at fastdds_lan.xml directly -- the placeholder makes Fast DDS fail to parse.

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
    _dds_out="${XDG_RUNTIME_DIR:-/tmp}/fastdds_active.xml"
    if sed "s/@LAN_IP@/$_dds_ip/g" "$_dds_template" > "$_dds_out"; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="$_dds_out"
        echo "setup_dds: iface=$_dds_iface ip=$_dds_ip -> $_dds_out"
    else
        echo "setup_dds: failed to render $_dds_out" >&2
    fi
fi

unset _dds_dir _dds_template _dds_iface _dds_ip _dds_out
