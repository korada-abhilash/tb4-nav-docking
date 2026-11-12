#!/usr/bin/env python3
"""Navigation on our RTAB-Map map: N random goals (seeded) across its free space, sent to Nav2 one after another.

Goals are sampled from the saved map's free cells with clearance from walls and at least MIN_HOP metres
from the previous goal. Records per goal: result, sim time, distance driven (from /odom).

Usage (in the container, sim + Nav2 bringup with AMCL on our map running):
    python3 /ws/scripts/nav_eval.py [map_yaml=results/maps/warehouse_rtabmap.yaml] [n=20] [seed=0]
Writes /ws/outputs/nav_eval.json.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from PIL import Image
from scipy import ndimage

MAP = Path(sys.argv[1] if len(sys.argv) > 1 else "/ws/results/maps/warehouse_rtabmap.yaml")
N = int(sys.argv[2]) if len(sys.argv) > 2 else 20
SEED = int(sys.argv[3]) if len(sys.argv) > 3 else 0
CLEARANCE, MIN_HOP, GOAL_TIMEOUT = 0.8, 5.0, 240.0


def sample_goals():
    meta = yaml.safe_load(MAP.read_text())
    img = np.array(Image.open(MAP.parent / meta["image"]).convert("L")).astype(np.float32)
    free = (255 - img) / 255.0 < meta["free_thresh"]
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    h = img.shape[0]
    dist = ndimage.distance_transform_edt(free) * res
    labels, _ = ndimage.label(free)
    r0, c0 = h - 1 - int(round(-oy / res)), int(round(-ox / res))  # start pose = map origin (0, 0)
    ok = np.argwhere((labels == labels[r0, c0]) & (dist >= CLEARANCE))
    rng = np.random.default_rng(SEED)
    goals, prev = [], (0.0, 0.0)
    while len(goals) < N:
        r, c = ok[rng.integers(len(ok))]
        x, y = ox + c * res, oy + (h - 1 - r) * res
        if math.hypot(x - prev[0], y - prev[1]) >= MIN_HOP:
            goals.append((round(float(x), 2), round(float(y), 2), round(float(rng.uniform(-math.pi, math.pi)), 2)))
            prev = (x, y)
    return goals


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    odom = {"last": None, "dist": 0.0}

    def on_odom(m):
        p = m.pose.pose.position
        if odom["last"] is not None:
            odom["dist"] += math.hypot(p.x - odom["last"][0], p.y - odom["last"][1])
        odom["last"] = (p.x, p.y)

    nav.create_subscription(Odometry, "/odom", on_odom, 20)
    nav.setInitialPose(pose(nav, 0.0, 0.0, 0.0))  # robot spawns at the map origin (RTAB-Map start frame)
    nav.waitUntilNav2Active()

    goals, log = sample_goals(), []
    for i, (x, y, yaw) in enumerate(goals):
        d0, t0 = odom["dist"], nav.get_clock().now()
        nav.goToPose(pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)
            if (nav.get_clock().now() - t0).nanoseconds / 1e9 > GOAL_TIMEOUT:
                nav.cancelTask()
                break
        res = str(nav.getResult()).split(".")[-1]
        sim_s = (nav.get_clock().now() - t0).nanoseconds / 1e9
        log.append({"i": i, "goal": [x, y, yaw], "result": res, "sim_s": round(sim_s, 1),
                    "driven_m": round(odom["dist"] - d0, 2)})
        print(f"[{i + 1}/{N}] ({x:.1f}, {y:.1f}) {res} in {sim_s:.0f} s, {log[-1]['driven_m']:.1f} m", flush=True)

    ok = [g for g in log if g["result"] == "SUCCEEDED"]
    summary = {"map": str(MAP), "seed": SEED, "goals": N, "succeeded": len(ok),
               "mean_sim_s": round(float(np.mean([g["sim_s"] for g in ok])), 1) if ok else None,
               "total_driven_m": round(sum(g["driven_m"] for g in log), 1), "log": log}
    json.dump(summary, open("/ws/outputs/nav_eval.json", "w"), indent=2)
    print(f"done: {len(ok)}/{N} goals reached, {summary['total_driven_m']} m driven", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
