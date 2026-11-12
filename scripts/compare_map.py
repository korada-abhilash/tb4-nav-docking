#!/usr/bin/env python3
"""Score an RTAB-Map occupancy map against the warehouse's stock (ground-truth) map.

Both maps are compared in the world frame (ours is in RTAB-Map's start frame; the spawn pose from
config/sim.env maps it back). Metrics:
  - wall accuracy: distance from each occupied cell in our map to the nearest occupied cell of the
    stock map (median, 95th percentile, share within 10 cm)
  - coverage: share of the stock map's free floor reachable from the spawn point that our map marks free
Also writes an overlay PNG (grey = stock walls, red = our walls, light = our free space).

Usage (in the container): python3 /ws/scripts/compare_map.py outputs/maps/warehouse_rtabmap.yaml [out.png]
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
sx, sy, syaw = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])


def load(path):
    meta = yaml.safe_load(Path(path).read_text())
    img = np.array(Image.open(Path(path).parent / meta["image"]).convert("L")).astype(np.float32)
    p = (255 - img) / 255.0
    occ, free = p > meta["occupied_thresh"], p < meta["free_thresh"]
    return meta, occ, free


gt_meta, gt_occ, gt_free = load(env["STOCK_MAP"])
our_meta, our_occ, our_free = load(sys.argv[1])
gres, (gox, goy, _) = gt_meta["resolution"], gt_meta["origin"]
gh, gw = gt_occ.shape
ores, (oox, ooy, _) = our_meta["resolution"], our_meta["origin"]
oh, ow = our_occ.shape


def our_cells_to_world(rows, cols):
    xs, ys = oox + cols * ores, ooy + (oh - 1 - rows) * ores          # start frame
    c, s = math.cos(syaw), math.sin(syaw)
    return sx + c * xs - s * ys, sy + s * xs + c * ys                   # world frame


def world_to_gt(x, y):
    return (gh - 1 - np.round((y - goy) / gres)).astype(int), np.round((x - gox) / gres).astype(int)


# wall accuracy
dist_to_gt_wall = ndimage.distance_transform_edt(~gt_occ) * gres
r, c = np.nonzero(our_occ)
gr, gc = world_to_gt(*our_cells_to_world(r, c))
inside = (gr >= 0) & (gr < gh) & (gc >= 0) & (gc < gw)
d = dist_to_gt_wall[gr[inside], gc[inside]]

# coverage of reachable stock floor
labels, _ = ndimage.label(gt_free)
sr, sc = world_to_gt(np.array([sx]), np.array([sy]))
reach = labels == labels[sr[0], sc[0]]
covered = np.zeros_like(reach)
fr, fc = np.nonzero(our_free)
fgr, fgc = world_to_gt(*our_cells_to_world(fr, fc))
ok = (fgr >= 0) & (fgr < gh) & (fgc >= 0) & (fgc < gw)
covered[fgr[ok], fgc[ok]] = True
covered = ndimage.binary_dilation(covered, iterations=1) & reach  # our 5 cm cells vs the stock 3 cm grid

res = {
    "our_wall_cells": int(inside.sum()),
    "wall_err_median_m": round(float(np.median(d)), 3),
    "wall_err_p95_m": round(float(np.percentile(d, 95)), 3),
    "wall_within_10cm": round(float((d <= 0.10).mean()), 3),
    "floor_coverage": round(float(covered.sum() / reach.sum()), 3),
    "reachable_floor_m2": round(float(reach.sum() * gres * gres), 1),
}
print(json.dumps(res, indent=2))
out_json = Path(sys.argv[1]).with_suffix(".score.json")
out_json.write_text(json.dumps(res, indent=2))

# overlay image in the stock map's grid
rgb = np.full((gh, gw, 3), 255, np.uint8)
rgb[covered] = (225, 236, 250)
rgb[gt_occ] = (150, 150, 150)
rgb[gr[inside], gc[inside]] = (220, 40, 40)
png = sys.argv[2] if len(sys.argv) > 2 else str(Path(sys.argv[1]).with_suffix(".overlay.png"))
Image.fromarray(rgb).save(png)
print("wrote", out_json, "and", png)
