#!/usr/bin/env python3
"""Nav2 params for the docking trials, derived from nav2_bringup's stock nav2_params.yaml.

Adds our dock instance ('home_dock' at DOCK_MAP_* in the map frame) and sets the SimpleChargingDock
detection mode:
  apriltag : use_external_detection_pose = true, with the external_detection_* offsets measured by
             calibrate_tag.py (config/tag_calibration.yaml); the final approach follows the detected tag
  map_only : use_external_detection_pose = false; the dock pose comes only from the map, so accuracy is
             bounded by AMCL localization
Both modes: docking_threshold 0.05 -> 0.015 m. With no charging contact to sense (use_battery_status is false
in simulation), the server ends docking when its estimate is within this distance of the dock; the stock
5 cm let the robot stop ~5 cm short. Everything else is stock. Usage: python3 make_nav2_docking_params.py <apriltag|map_only>
Writes /ws/config/nav2_docking_<mode>.yaml
"""
import sys
from pathlib import Path

import yaml

mode = sys.argv[1]
assert mode in ("apriltag", "map_only"), mode
env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
p = yaml.safe_load(open("/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"))
d = p["docking_server"]["ros__parameters"]
dock = d["simple_charging_dock"]
dock["use_external_detection_pose"] = mode == "apriltag"
dock["docking_threshold"] = 0.015
if mode == "apriltag":
    dock.update(yaml.safe_load(Path("/ws/config/tag_calibration.yaml").read_text())["docking_server"])
d["docks"] = ["home_dock"]
d["home_dock"] = {"type": "simple_charging_dock", "frame": "map",
                  "pose": [float(env["DOCK_MAP_X"]), float(env["DOCK_MAP_Y"]), float(env["DOCK_MAP_YAW"])]}
out = Path(f"/ws/config/nav2_docking_{mode}.yaml")
yaml.safe_dump(p, open(out, "w"), sort_keys=False)
print(f"wrote {out}: use_external_detection_pose={dock['use_external_detection_pose']}")
