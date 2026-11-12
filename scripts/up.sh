#!/usr/bin/env bash
# Start the headless sim (Nav2 minimal TurtleBot 4, warehouse) in a detached container named tb4-sim.
# Runs as the host user so files written to the mounted repo stay yours. DDS uses UDP only (no host IPC
# namespace, no shared memory), so stale /dev/shm segments from other runs cannot break ROS services.
# Usage: scripts/up.sh   — settings in config/sim.env; log in outputs/sim.log
set -euo pipefail
cd "$(dirname "$0")/.."
source config/sim.env
mkdir -p outputs
docker rm -f tb4-sim >/dev/null 2>&1 || true
docker run -d --name tb4-sim --net=host -u "$(id -u):$(id -g)" -e HOME=/tmp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --device /dev/dri --group-add video --group-add "$(getent group render | cut -d: -f3)" \
  -e DISPLAY -e XAUTHORITY=/tmp/.Xauthority -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "${XAUTHORITY:-$HOME/.Xauthority}":/tmp/.Xauthority:ro \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -v "$PWD":/ws tb4-nav-docking:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch nav2_minimal_tb4_sim simulation.launch.py headless:=True \
    world:=$WORLD x_pose:=$SPAWN_X y_pose:=$SPAWN_Y yaw:=$SPAWN_YAW > /ws/outputs/sim.log 2>&1" >/dev/null
echo "tb4-sim started; waiting for /scan..."
until docker exec tb4-sim bash -c "source /opt/ros/jazzy/setup.bash; timeout 5 ros2 topic list 2>/dev/null | grep -qx /scan"; do sleep 3; done
echo ready
