#!/bin/bash
# =============================================================================
# Run this script on your LAPTOP to start the 2D Path Viewer.
# It connects to the Duckiebot's sensor_fusion_node via TCP (port 9998)
# and displays a bird's-eye view of the trajectories.
#
# No ROS installation required!
#
# Requirements (pip install):
#   - numpy
#   - opencv-python
#
# Usage:
#   ./run_path_viewer.sh [DUCKIEBOT_IP]
# =============================================================================

DUCKIEBOT_IP="${1:-192.168.0.166}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================================================"
echo " Duckiebot 2D Path Viewer (Sensor Fusion)"
echo "================================================"
echo " Duckiebot IP: ${DUCKIEBOT_IP}"
echo " Port:         9998"
echo " Blue=Odometry  Green=ORB-SLAM  Red=Fused(EKF)"
echo "================================================"
echo ""

# Activate venv if available
if [ -f "$HOME/ORBSLAM/venv/bin/activate" ]; then
    source "$HOME/ORBSLAM/venv/bin/activate"
    echo "Activated venv: $HOME/ORBSLAM/venv"
fi

python3 "${SCRIPT_DIR}/path_viewer_standalone.py" "${DUCKIEBOT_IP}"
