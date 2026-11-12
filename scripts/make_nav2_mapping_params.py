#!/usr/bin/env python3
"""Derive the Nav2 params used *while mapping* from nav2_bringup's stock nav2_params.yaml.

While SLAM is still growing the map, a global costmap sized to that map rejects goals outside the explored
area. During mapping only, the global costmap therefore drops the static layer and uses a fixed extent
covering the warehouse (the stock map's bounds, transformed into RTAB-Map's start frame), built from live
lidar obstacles with unknown space allowed. Everything else is the stock config.
Writes /ws/config/nav2_mapping.yaml.
"""
import math
from pathlib import Path

import yaml
from PIL import Image

env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
sx, sy, syaw = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])
meta = yaml.safe_load(Path(env["STOCK_MAP"]).read_text())
w_px, h_px = Image.open(Path(env["STOCK_MAP"]).parent / meta["image"]).size
(ox, oy, _), res = meta["origin"], meta["resolution"]

# stock map corners (world) -> start frame, then the axis-aligned box around them (+1 m margin)
corners = [(ox, oy), (ox + w_px * res, oy), (ox, oy + h_px * res), (ox + w_px * res, oy + h_px * res)]
c, s = math.cos(-syaw), math.sin(-syaw)
pts = [(c * (x - sx) - s * (y - sy), s * (x - sx) + c * (y - sy)) for x, y in corners]
xmin, xmax = min(p[0] for p in pts) - 1, max(p[0] for p in pts) + 1
ymin, ymax = min(p[1] for p in pts) - 1, max(p[1] for p in pts) + 1

p = yaml.safe_load(open("/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"))
g = p["global_costmap"]["global_costmap"]["ros__parameters"]
g["plugins"] = ["obstacle_layer", "inflation_layer"]
g.pop("static_layer", None)
g.update({"rolling_window": False, "width": int(math.ceil(xmax - xmin)), "height": int(math.ceil(ymax - ymin)),
          "origin_x": round(xmin, 2), "origin_y": round(ymin, 2), "track_unknown_space": True})
g["obstacle_layer"]["scan"].update({"raytrace_max_range": 6.0, "obstacle_max_range": 5.5})
yaml.safe_dump(p, open("/ws/config/nav2_mapping.yaml", "w"), sort_keys=False)
print(f"wrote /ws/config/nav2_mapping.yaml: global costmap {g['width']} x {g['height']} m at ({g['origin_x']}, {g['origin_y']})")
