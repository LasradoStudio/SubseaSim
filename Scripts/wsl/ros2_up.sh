#!/usr/bin/env bash
# Start one piece of the ROS 2 side inside WSL:  ros2_up.sh bridge [port] | ros2_up.sh rviz
# The bridge connects to the simulation on the Windows host, which WSL (NAT mode) reaches through
# its default gateway. Override with SUBSEA_HOST.
set -eo pipefail
root="${SUBSEA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export PYTHONPATH="$root/Bridge:${PYTHONPATH}"

case "${1:-bridge}" in
    bridge)
        host="${SUBSEA_HOST:-$(ip route | awk '/default/ {print $3; exit}')}"
        exec python3 -m subsea_bridge.ros2_bridge --ros-args -r __ns:=/subsea \
            -p host:="$host" -p port:="${2:-9870}"
        ;;
    rviz)
        # Draw on the GPU through WSLg's D3D12 driver (Mesa otherwise falls back to llvmpipe, ~2 fps
        # with a 20 s point-cloud trail). Prefer the NVIDIA adapter when there is one.
        export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
        export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
        exec rviz2 -d "$root/Bridge/docker/rviz/subsea.rviz"
        ;;
    *)
        echo "usage: ros2_up.sh bridge [port] | rviz" >&2; exit 2
        ;;
esac
