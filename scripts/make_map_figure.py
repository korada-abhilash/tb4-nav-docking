#!/usr/bin/env python3
"""Three-panel figure of map quality: (1) the warehouse's true layout, (2) the RTAB-Map map built by the
robot, (3) our walls coloured by their distance to the nearest true wall.

Uses the same frame alignment and wall-error definition as compare_map.py, and its saved score for the title.
Usage (in the container): python3 /ws/scripts/make_map_figure.py [our_map_yaml] [out_png]
"""
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from PIL import Image  # noqa: E402
from scipy import ndimage  # noqa: E402

OUR = Path(sys.argv[1] if len(sys.argv) > 1 else "/ws/results/maps/warehouse_rtabmap.yaml")
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else "/ws/docs/map_figure.png")
INK, INK2, SURFACE = "#0b0b0b", "#52514e", "#ffffff"
TRUE_WALL, OUR_WALL, OUR_FLOOR = "#5f5e5a", "#1f1f1e", "#dce9fa"
ERR_COLORS = ["#2a78d6", "#eda100", "#e34948"]  # <=5 cm, 5-10 cm, >10 cm
ERR_LABELS = ["within 5 cm", "5–10 cm", "over 10 cm"]

env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
sx, sy, syaw = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])


def load(path):
    meta = yaml.safe_load(Path(path).read_text())
    img = np.array(Image.open(Path(path).parent / meta["image"]).convert("L")).astype(np.float32)
    p = (255 - img) / 255.0
    return meta, p > meta["occupied_thresh"], p < meta["free_thresh"]


gt_meta, gt_occ, gt_free = load(env["STOCK_MAP"])
our_meta, our_occ, our_free = load(OUR)
gres, (gox, goy, _) = gt_meta["resolution"], gt_meta["origin"]
gh, gw = gt_occ.shape
ores, (oox, ooy, _) = our_meta["resolution"], our_meta["origin"]
oh = our_occ.shape[0]


def our_to_gt_cells(mask):
    """Our map cells (start frame) -> the ground-truth grid (world frame)."""
    r, c = np.nonzero(mask)
    xs, ys = oox + c * ores, ooy + (oh - 1 - r) * ores
    wx = sx + math.cos(syaw) * xs - math.sin(syaw) * ys
    wy = sy + math.sin(syaw) * xs + math.cos(syaw) * ys
    gr = (gh - 1 - np.round((wy - goy) / gres)).astype(int)
    gc = np.round((wx - gox) / gres).astype(int)
    ok = (gr >= 0) & (gr < gh) & (gc >= 0) & (gc < gw)
    return gr[ok], gc[ok]


wr, wc = our_to_gt_cells(our_occ)
fr, fc = our_to_gt_cells(our_free)
our_wall = np.zeros_like(gt_occ)
our_wall[wr, wc] = True
our_floor = np.zeros_like(gt_occ)
our_floor[fr, fc] = True
our_floor = ndimage.binary_closing(ndimage.binary_dilation(our_floor, iterations=1), iterations=2)

# wall error: distance from each of our wall cells to the nearest true wall
dist = ndimage.distance_transform_edt(~gt_occ) * gres
err_class = np.full(gt_occ.shape, -1, int)
d = dist[wr, wc]
err_class[wr, wc] = np.digitize(d, [0.05, 0.10])  # 0: <=5 cm, 1: 5-10 cm, 2: >10 cm

# thicken lines for display only (3 cm cells -> ~15 cm strokes), and crop to the occupied extent
thick = lambda m: ndimage.binary_dilation(m, iterations=2)  # noqa: E731
rows = np.nonzero(gt_occ.any(1) | our_wall.any(1))[0]
cols = np.nonzero(gt_occ.any(0) | our_wall.any(0))[0]
crop = (slice(max(rows[0] - 10, 0), rows[-1] + 11), slice(max(cols[0] - 10, 0), cols[-1] + 11))


def rgba(mask, color):
    img = np.zeros(mask.shape + (4,))
    img[mask] = matplotlib.colors.to_rgba(color)
    return img[crop]


score = json.loads(OUR.with_suffix(".score.json").read_text())
fig, axes = plt.subplots(1, 3, figsize=(13, 7.6), facecolor=SURFACE)
titles = ["Ground truth: the warehouse layout", "Built by the robot: RTAB-Map", "Our walls vs. the true walls"]
for ax, t in zip(axes, titles):
    ax.set_facecolor(SURFACE)
    ax.set_title(t, color=INK, fontsize=12, loc="left", pad=8)
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)

axes[0].imshow(rgba(thick(gt_occ), TRUE_WALL), interpolation="nearest")
axes[1].imshow(rgba(our_floor & ~our_wall, OUR_FLOOR), interpolation="nearest")
axes[1].imshow(rgba(thick(our_wall), OUR_WALL), interpolation="nearest")
axes[2].imshow(rgba(thick(gt_occ), "#d9d8d4"), interpolation="nearest")
for k in range(3):  # draw the larger errors last so they stay visible
    axes[2].imshow(rgba(thick(err_class == k), ERR_COLORS[k]), interpolation="nearest")

LEG = dict(loc="upper center", bbox_to_anchor=(0.5, -0.01), frameon=False, fontsize=9.5, labelcolor=INK, ncol=2)
axes[0].legend(handles=[Patch(color=TRUE_WALL, label="true walls and shelves")], **LEG)
axes[1].legend(handles=[Patch(color=OUR_WALL, label="walls the robot mapped"),
                        Patch(facecolor=OUR_FLOOR, edgecolor="#9fb8d8", label="floor the robot saw")], **LEG)
shares = [float((err_class[wr, wc] == k).mean()) for k in range(3)]
axes[2].legend(handles=[Patch(color=c, label=f"{l} ({s:.0%})") for c, l, s in zip(ERR_COLORS, ERR_LABELS, shares)]
               + [Patch(color="#d9d8d4", label="true walls")], **LEG)

# 5 m scale bar
px = 5.0 / gres
h = crop[0].stop - crop[0].start
for ax in axes:
    ax.plot([15, 15 + px], [h - 25, h - 25], color=INK, lw=3, solid_capstyle="butt")
    ax.text(15 + px / 2, h - 40, "5 m", ha="center", va="bottom", color=INK, fontsize=9)

fig.suptitle(f"Map accuracy: median wall error {score['wall_err_median_m'] * 100:.0f} cm, "
             f"{score['wall_within_10cm']:.0%} of walls within 10 cm, {score['floor_coverage']:.0%} of the floor mapped",
             x=0.01, ha="left", color=INK, fontsize=14)
fig.text(0.01, 0.012, "The unmapped area at the top-left is a closed room the robot could not enter. Walls are drawn "
         "thicker than their 3 cm cells for visibility.", color=INK2, fontsize=9)
fig.tight_layout(rect=(0, 0.04, 1, 0.965))
OUT.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT, dpi=110, facecolor=SURFACE)
print("wrote", OUT, "shares", [round(s, 3) for s in shares])
