#!/usr/bin/env bash
# Start (or attach to) the dev container with GUI and Intel GPU access. The repo is mounted at /ws.
# Usage: scripts/run.sh [command...]   (default: interactive bash)
set -euo pipefail
cd "$(dirname "$0")/.."
NAME=tb4-sim
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  exec docker exec -it "$NAME" bash -ic "${*:-bash}"
fi
exec docker run --rm -it --name "$NAME" \
  --net=host --ipc=host \
  --device /dev/dri --group-add video --group-add "$(getent group render | cut -d: -f3)" \
  -e DISPLAY -e XAUTHORITY=/tmp/.Xauthority -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v "${XAUTHORITY:-$HOME/.Xauthority}":/tmp/.Xauthority:ro \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  -v "$PWD":/ws \
  tb4-nav-docking:jazzy bash -ic "${*:-bash}"
