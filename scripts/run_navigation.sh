#!/usr/bin/env bash
# Navigation run on our saved RTAB-Map map: fresh sim -> Nav2 bringup (map_server + AMCL) -> nav_eval goals.
# Usage: scripts/run_navigation.sh [n_goals=20] [seed=0]. Logs and results go to outputs/.
set -euo pipefail
cd "$(dirname "$0")/.."
N=${1:-20}; SEED=${2:-0}
scripts/up.sh
docker exec -d -u "$(id -u):$(id -g)" -e HOME=/tmp tb4-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true slam:=False map:=/ws/results/maps/warehouse_rtabmap.yaml > /ws/outputs/nav2_navigation.log 2>&1"
scripts/exec.sh "python3 -u scripts/nav_eval.py results/maps/warehouse_rtabmap.yaml $N $SEED > outputs/nav_eval.log 2>&1"
tail -1 outputs/nav_eval.log
