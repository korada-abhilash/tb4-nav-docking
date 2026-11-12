#!/usr/bin/env bash
# Spawn the AprilTag dock into the running sim at DOCK_MAP_* (config/sim.env), converted to the world frame.
set -euo pipefail
cd "$(dirname "$0")/.."
source config/sim.env
read WX WY WYAW < <(python3 -c "
import math; sx,sy,syaw=$SPAWN_X,$SPAWN_Y,$SPAWN_YAW; x,y,yaw=$DOCK_MAP_X,$DOCK_MAP_Y,$DOCK_MAP_YAW
print(round(sx+math.cos(syaw)*x-math.sin(syaw)*y,4), round(sy+math.sin(syaw)*x+math.cos(syaw)*y,4), round(syaw+yaw,4))")
echo "dock world pose: $WX $WY $WYAW"
scripts/exec.sh "ros2 run ros_gz_sim create -world warehouse -file /ws/models/dock_apriltag/model.sdf -name dock -x $WX -y $WY -z 0 -Y $WYAW" | tail -1
