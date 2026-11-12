#!/usr/bin/env bash
# Full mapping run from a fresh sim: sim -> RTAB-Map SLAM -> Nav2 (mapping params) -> waypoint tour -> save map.
# Usage: scripts/run_mapping.sh. Logs, the RTAB-Map database and the 2D map go to outputs/.
set -euo pipefail
cd "$(dirname "$0")/.."
scripts/up.sh
scripts/exec.sh "python3 scripts/make_waypoints.py 4.0 0.7 && python3 scripts/make_nav2_mapping_params.py"
docker exec -d -u "$(id -u):$(id -g)" -e HOME=/tmp tb4-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch /ws/launch/slam_rtabmap.launch.py database_path:=/ws/outputs/rtabmap_warehouse.db > /ws/outputs/rtabmap_mapping.log 2>&1"
until scripts/exec.sh "timeout 5 ros2 topic info /map 2>/dev/null" | grep "Publisher count: [1-9]" >/dev/null; do sleep 5; done  # no grep -q: with pipefail its early exit SIGPIPEs the writer
echo "RTAB-Map publishing /map"
docker exec -d -u "$(id -u):$(id -g)" -e HOME=/tmp tb4-sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/ws/config/nav2_mapping.yaml > /ws/outputs/nav2_mapping.log 2>&1"
until grep -q "Managed nodes are active" outputs/nav2_mapping.log 2>/dev/null; do sleep 5; done
echo "Nav2 active; starting tour"
scripts/exec.sh "python3 -u scripts/map_tour.py > outputs/map_tour.log 2>&1"
mkdir -p outputs/maps
scripts/exec.sh "ros2 run nav2_map_server map_saver_cli -f outputs/maps/warehouse_rtabmap -t /map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=20.0" | tail -2
echo "mapping run finished"
