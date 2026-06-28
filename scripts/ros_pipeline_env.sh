#!/usr/bin/env bash
# Shared ROS 2 environment for RealSense + ORB-SLAM3 pipeline.
set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_ROOT="$(cd "${_SCRIPT_DIR}/.." && pwd)"
export ROS_WS_ROOT

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="file://${ROS_WS_ROOT}/cyclonedds.xml"

# Solo-Pi pipeline: force one discovery profile on every terminal.
# Mismatched SUBNET vs LOCALHOST makes RealSense topics invisible to ORB3.
unset ROS_STATIC_PEERS 2>/dev/null || true
unset ROS_LOCALHOST_ONLY 2>/dev/null || true
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

_ORB_PKG="${ROS_WS_ROOT}/src/ros2_orb_slam3/orb_slam3"
export ORBSLAM3_VOCAB="${ORBSLAM3_VOCAB:-${_ORB_PKG}/Vocabulary/ORBvoc.txt}"
export ORBSLAM3_CONFIG="${ORBSLAM3_CONFIG:-${_ORB_PKG}/config/RGB-D/RealSense_D455.yaml}"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${ROS_WS_ROOT}/install/setup.bash" ]]; then
  source "${ROS_WS_ROOT}/install/setup.bash"
else
  echo "WARN: ${ROS_WS_ROOT}/install/setup.bash not found — run colcon build first" >&2
fi
set -u
