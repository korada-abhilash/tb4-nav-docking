#!/usr/bin/env python3
"""Summarize docking trial files into results/docking/summary.json and a markdown table.

Reports, per trial set: docking-action success, final error vs ground truth (median |.|, 90th percentile,
mean, SD) in depth (x), sideways (y) and heading, and how many trials ended within the position tolerance
(|x| <= 3 cm, |y| <= 2 cm) and within the full tolerance (position and |heading| <= 3 deg).

Usage: python3 scripts/docking_report.py   (reads results/docking/dock_trials_*.json)
"""
import json
import statistics as st
from pathlib import Path

SETS = [
    ("map_only_seed0", "Map pose only (AMCL)", "seed 0"),
    ("apriltag_seed0", "AprilTag, calibrated at 0.7 m", "seed 0"),
    ("map_only_seed1", "Map pose only (AMCL)", "seed 1"),
    ("apriltag_corrected_seed1", "AprilTag, + position bias correction (fitted on seed 0)", "seed 1"),
]
D = Path("results/docking")


def stats(v):
    a = sorted(abs(x) for x in v)
    return {"median_abs": round(st.median(a), 4), "p90_abs": round(a[int(0.9 * len(a)) - 1], 4),
            "mean": round(st.mean(v), 4), "sd": round(st.pstdev(v), 4)}


summary, rows = {}, []
for key, label, seed in SETS:
    f = D / f"dock_trials_{key}.json"
    if not f.exists():
        continue
    t = json.load(open(f))["trials"]
    n = len(t)
    pos_ok = [abs(e["err_x_m"]) <= 0.03 and abs(e["err_y_m"]) <= 0.02 for e in t]
    full_ok = [p and abs(e["err_yaw_deg"]) <= 3 for p, e in zip(pos_ok, t)]
    s = {"label": label, "starts": seed, "n": n, "action_success": sum(e["action_success"] for e in t),
         "within_position_tol": sum(pos_ok), "within_full_tol": sum(full_ok),
         "x_m": stats([e["err_x_m"] for e in t]), "y_m": stats([e["err_y_m"] for e in t]),
         "yaw_deg": stats([e["err_yaw_deg"] for e in t])}
    summary[key] = s
    rows.append(f"| {label} | {seed} | {s['action_success']}/{n} | {s['x_m']['median_abs'] * 100:.1f} "
                f"({s['x_m']['p90_abs'] * 100:.1f}) | {s['y_m']['median_abs'] * 100:.1f} ({s['y_m']['p90_abs'] * 100:.1f}) "
                f"| {s['yaw_deg']['median_abs']:.1f} ({s['yaw_deg']['p90_abs']:.1f}) | {s['within_position_tol']}/{n} "
                f"| {s['within_full_tol']}/{n} |")

table = "\n".join([
    "| Dock pose source | Start poses | Action reports success | Depth error, cm: median (p90) | "
    "Sideways error, cm: median (p90) | Heading error, deg: median (p90) | Within ±3 cm / ±2 cm | "
    "Also within ±3° |",
    "|---|---|---|---|---|---|---|---|", *rows])
json.dump(summary, open(D / "summary.json", "w"), indent=2)
(D / "table.md").write_text(table + "\n")
print(table)
