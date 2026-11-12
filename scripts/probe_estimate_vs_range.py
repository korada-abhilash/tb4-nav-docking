#!/usr/bin/env python3
"""Dock-estimate error vs distance: teleport the robot to exact poses on the dock axis (and with lateral /
heading offsets), then compare the docking server's dock estimate (same math as check_dock_estimate.py)
with ground truth. Prints position and yaw error of the estimate at each pose.

Usage (in the container; sim, dock, apriltag_node, relay and Nav2/AMCL TF running): python3 probe_estimate_vs_range.py
"""
import math
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, "/ws/scripts")
env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
sx, sy, syaw = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])
dx, dy, dyaw = float(env["DOCK_MAP_X"]), float(env["DOCK_MAP_Y"]), float(env["DOCK_MAP_YAW"])
DWX, DWY = sx + math.cos(syaw) * dx - math.sin(syaw) * dy, sy + math.sin(syaw) * dx + math.cos(syaw) * dy
DWYAW = syaw + dyaw


def teleport(back, lateral, dyaw_deg):
    """Robot base at (-back, lateral) in the dock frame, heading dock yaw + dyaw."""
    x = DWX + math.cos(DWYAW) * -back - math.sin(DWYAW) * lateral
    y = DWY + math.sin(DWYAW) * -back + math.cos(DWYAW) * lateral
    yaw = DWYAW + math.radians(dyaw_deg)
    req = (f'name: "turtlebot4", position: {{x: {x}, y: {y}, z: 0.0}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    subprocess.run(["gz", "service", "-s", "/world/warehouse/set_pose", "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req], capture_output=True)


for back, lat, dyaw_deg in [(0.7, 0, 0), (0.4, 0, 0), (0.2, 0, 0), (0.05, 0, 0), (0.05, 0.03, 0), (0.05, 0, 5),
                            (0.4, 0, 8)]:
    teleport(back, lat, dyaw_deg)
    time.sleep(3)
    out = subprocess.run(["python3", "/ws/scripts/check_dock_estimate.py"], capture_output=True, text=True).stdout
    line = next((l for l in out.splitlines() if l.startswith("tag*R_ext")), "no estimate")
    nums = [float(v) for v in __import__("re").findall(r"=(-?[\d.]+)", line)]
    if len(nums) == 6:
        ex, ey, eyaw, tx, ty, tyaw = nums
        print(f"robot at back={back:.2f} m lat={lat:+.2f} m dyaw={dyaw_deg:+d} deg -> estimate error: "
              f"pos {math.hypot(ex - tx, ey - ty) * 100:.1f} cm, yaw {eyaw - tyaw:+.1f} deg", flush=True)
    else:
        print(f"robot at back={back:.2f} lat={lat:+.2f} dyaw={dyaw_deg:+d}: {line}", flush=True)
