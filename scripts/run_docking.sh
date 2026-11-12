#!/usr/bin/env bash
# One docking trial set from a fresh sim: sim -> dock -> (AprilTag detector + relay) -> Nav2 on our map ->
# dock_trials. Usage: scripts/run_docking.sh <apriltag|map_only> [n=20] [seed=0]
set -euo pipefail
cd "$(dirname "$0")/.."
MODE=$1; N=${2:-20}; SEED=${3:-0}
RUN="docker exec -d -u $(id -u):$(id -g) -e HOME=/tmp tb4-sim bash -c"
scripts/up.sh
scripts/spawn_dock.sh
scripts/exec.sh "python3 scripts/make_nav2_docking_params.py $MODE"
if [ "$MODE" = apriltag ]; then
  $RUN "source /opt/ros/jazzy/setup.bash && ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/rgbd_camera/image -r camera_info:=/rgbd_camera/camera_info --params-file /ws/config/apriltag.yaml -p use_sim_time:=true > /ws/outputs/apriltag.log 2>&1"
  $RUN "source /opt/ros/jazzy/setup.bash && python3 -u /ws/scripts/dock_pose_relay.py > /ws/outputs/relay.log 2>&1"
fi
$RUN "source /opt/ros/jazzy/setup.bash && ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true slam:=False map:=/ws/results/maps/warehouse_rtabmap.yaml params_file:=/ws/config/nav2_docking_$MODE.yaml > /ws/outputs/nav2_docking_$MODE.log 2>&1"
scripts/exec.sh "python3 -u scripts/dock_trials.py --label $MODE --n $N --seed $SEED > outputs/dock_trials_$MODE.log 2>&1"
tail -1 outputs/dock_trials_$MODE.log
