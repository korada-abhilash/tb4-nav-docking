#!/usr/bin/env python3
"""Pick a mapping tour through the warehouse from the stock occupancy map's free space.

The stock map (nav2_bringup/maps/warehouse.*) is used only to choose reachable waypoints; the map we
report is built by RTAB-Map during the tour. Waypoints lie on a grid, keep a clearance from obstacles, and
are ordered greedily (nearest next) from the spawn pose.

The stock map is in the world frame. RTAB-Map's map frame starts at the robot's spawn pose, so waypoints
are written in that start frame: p_map = R(-yaw) (p_world - spawn).

Usage (in the container): python3 /ws/scripts/make_waypoints.py [spacing_m=4.0] [clearance_m=0.7]
Reads spawn pose from /ws/config/sim.env, writes /ws/config/mapping_waypoints.yaml.
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
sx, sy, syaw = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])
MAP = Path(env["STOCK_MAP"])
spacing = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
clearance = float(sys.argv[2]) if len(sys.argv) > 2 else 0.7

meta = yaml.safe_load(MAP.read_text())
img = np.array(Image.open(MAP.parent / meta["image"]))
res, (ox, oy, _) = meta["resolution"], meta["origin"]
free = (255 - img) / 255.0 < meta["free_thresh"]
dist = ndimage.distance_transform_edt(free) * res  # clearance (m) to the nearest non-free cell
h, w = img.shape


def world_to_cell(x, y):
    return h - 1 - int(round((y - oy) / res)), int(round((x - ox) / res))


def cell_to_world(r, c):
    return ox + c * res, oy + (h - 1 - r) * res


def world_to_start(x, y):
    dx, dy = x - sx, y - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    return c * dx - s * dy, s * dx + c * dy


labels, _ = ndimage.label(free)
reachable = labels == labels[world_to_cell(sx, sy)]  # free region connected to the spawn cell
step = int(round(spacing / res))
pts = [world_to_start(*cell_to_world(r, c)) for r in range(step // 2, h, step) for c in range(step // 2, w, step)
       if reachable[r, c] and dist[r, c] >= clearance]

tour, cur = [], (0.0, 0.0)
while pts:
    i = min(range(len(pts)), key=lambda k: (pts[k][0] - cur[0]) ** 2 + (pts[k][1] - cur[1]) ** 2)
    cur = pts.pop(i)
    tour.append([round(cur[0], 2), round(cur[1], 2)])
tour.append([0.0, 0.0])  # return to the start so the loop closes there

out = Path("/ws/config/mapping_waypoints.yaml")
out.write_text(yaml.safe_dump({"frame": "map (RTAB-Map, origin = spawn pose)", "spacing_m": spacing,
                               "clearance_m": clearance, "waypoints": tour}))
print(f"{len(tour)} waypoints from {MAP.name} ({w * res:.1f} x {h * res:.1f} m), written to {out}")
