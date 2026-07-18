#!/usr/bin/env bash
# One-click map saver. Saves the live SLAM /map to <name>.pgm + <name>.yaml.
# Uses a longer save_map_timeout so the latched /map sample is reliably received
# (the map_saver_cli default ~2s often misses it and errors "Failed to spin map subscription").
#
#   ./save_map.sh                 # -> maps/map_<default>.pgm/.yaml
#   ./save_map.sh maze_01         # -> maps/maze_01.pgm/.yaml
#   ./save_map.sh ~/foo/bar       # absolute/relative path also works
set -e
_here="$( cd "$( dirname "${BASH_SOURCE[0]:-$0}" )" && pwd )"

source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash"; fi
source "$_here/dds/setup_dds.sh"

name="${1:-my_map}"
# If a bare name (no slash) is given, drop it under the repo's maps/ dir.
case "$name" in
  */*) out="$name" ;;
  *)   mkdir -p "$_here/maps"; out="$_here/maps/$name" ;;
esac

echo "Saving map to ${out}.pgm / ${out}.yaml ..."
exec ros2 run nav2_map_server map_saver_cli -f "$out" --ros-args -p save_map_timeout:=10.0
