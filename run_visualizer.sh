#!/bin/bash
# =============================================================================
# Run this script on your LAPTOP to start the Pangolin visualizer.
# It connects to the Duckiebot via TCP and displays the 3D point cloud,
# camera trajectory, and annotated camera image.
#
# No ROS installation required!
#
# Requirements (pip install):
#   - numpy
#   - opencv-python
#   - PyOpenGL
#   - pangolin (already installed in your venv)
# =============================================================================

DUCKIEBOT_IP="${1:-192.168.0.166}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================================================"
echo " ORB-SLAM Standalone Visualizer (Pangolin)"
echo "================================================"
echo " Duckiebot IP: ${DUCKIEBOT_IP}"
echo " Port:         9999"
echo "================================================"
echo ""

# Activate venv if available
if [ -f "$HOME/ORBSLAM/venv/bin/activate" ]; then
    source "$HOME/ORBSLAM/venv/bin/activate"
    echo "Activated venv: $HOME/ORBSLAM/venv"
fi

python3 "${SCRIPT_DIR}/visualizer_standalone.py" "${DUCKIEBOT_IP}"
