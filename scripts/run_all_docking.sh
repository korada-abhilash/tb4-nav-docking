#!/usr/bin/env bash
# Both docking trial sets on the same seeded start poses: AprilTag, then map-only. Usage: scripts/run_all_docking.sh [n=20] [seed=0]
set -euo pipefail
cd "$(dirname "$0")/.."
for MODE in apriltag map_only; do scripts/run_docking.sh "$MODE" "${1:-20}" "${2:-0}"; done
docker rm -f tb4-sim >/dev/null
