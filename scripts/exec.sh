#!/usr/bin/env bash
# Run a command inside the running tb4-sim container (as the host user) with ROS sourced.
# Usage: scripts/exec.sh "<cmd>"
exec docker exec -u "$(id -u):$(id -g)" -e HOME=/tmp tb4-sim bash -c "source /opt/ros/jazzy/setup.bash && cd /ws && $*"
