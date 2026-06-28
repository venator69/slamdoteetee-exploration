#!/usr/bin/env bash
# Shared ROS 2 environment for RealSense + ORB-SLAM3 pipeline on this Pi.
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="file:///home/slamet/Dev/ros2_test/cyclonedds.xml"

# Solo-Pi pipeline: force one discovery profile on every terminal.
# Mismatched SUBNET vs LOCALHOST makes RealSense topics invisible to ORB3.
unset ROS_STATIC_PEERS 2>/dev/null || true
unset ROS_LOCALHOST_ONLY 2>/dev/null || true
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

export ORBSLAM3_VOCAB="${ORBSLAM3_VOCAB:-/home/slamet/Dev/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt}"
export ORBSLAM3_CONFIG="${ORBSLAM3_CONFIG:-/home/slamet/Dev/ros2_test/src/ros2_orb_slam3/orb_slam3/config/RGB-D/RealSense_D455.yaml}"

set +u
source /opt/ros/jazzy/setup.bash
source /home/slamet/Dev/ros2_test/install/setup.bash
set -u
