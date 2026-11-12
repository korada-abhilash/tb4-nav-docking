#!/usr/bin/env python3
"""Docking trials: drive to a random start pose in front of the dock, dock with Nav2's docking server,
measure the final pose error against Gazebo ground truth, undock, repeat.

Start poses (seeded): 1.5-3.5 m in front of the dock, within +-60 deg of its axis, random heading.
Error is the robot base pose expressed in the dock model frame (the frame where base_link should be
when docked): x = along the dock axis (+ = too far in), y = lateral, yaw. A trial counts as docked when
the action succeeds AND |x| <= 3 cm, |y| <= 2 cm, |yaw| <= 3 deg (charging-contact tolerance).

Usage (in the container; sim + dock + Nav2 with docking_server (+ apriltag_node and relay for the
apriltag mode) running):
    python3 /ws/scripts/dock_trials.py --label apriltag [--n 20] [--seed 0]
Writes /ws/outputs/dock_trials_<label>.json
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, UndockRobot
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
import sys
sys.path.insert(0, "/ws/scripts")
from gt_pose import gt_pose  # noqa: E402

TOL_X, TOL_Y, TOL_YAW = 0.03, 0.02, math.radians(3)
env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
SX, SY, SYAW = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])
DX, DY, DYAW = float(env["DOCK_MAP_X"]), float(env["DOCK_MAP_Y"]), float(env["DOCK_MAP_YAW"])
# dock pose in the world frame
DWX = SX + math.cos(SYAW) * DX - math.sin(SYAW) * DY
DWY = SY + math.sin(SYAW) * DX + math.cos(SYAW) * DY
DWYAW = SYAW + DYAW


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def run_action(nav, client, goal, timeout_s):
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(nav, fut)
    handle = fut.result()
    if not handle.accepted:
        return None, "rejected"
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(nav, res_fut, timeout_sec=timeout_s)
    if not res_fut.done():
        handle.cancel_goal_async()
        return None, "timeout"
    return res_fut.result().result, res_fut.result().status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    dock_client = ActionClient(nav, DockRobot, "/dock_robot")
    undock_client = ActionClient(nav, UndockRobot, "/undock_robot")
    nav.setInitialPose(pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active()
    dock_client.wait_for_server()
    undock_client.wait_for_server()

    def gt_error():
        rx, ry, ryaw = gt_pose()
        dx, dy = rx - DWX, ry - DWY
        c, s = math.cos(-DWYAW), math.sin(-DWYAW)
        return c * dx - s * dy, s * dx + c * dy, wrap(ryaw - DWYAW)

    rng = np.random.default_rng(args.seed)
    log = []
    for i in range(args.n):
        # start pose in front of the dock (dock frame: robot approaches along +x, so "in front" is -x)
        r, ang = rng.uniform(1.5, 3.5), rng.uniform(-math.radians(60), math.radians(60))
        lx, ly = -r * math.cos(ang), r * math.sin(ang)
        hx = DX + math.cos(DYAW) * lx - math.sin(DYAW) * ly
        hy = DY + math.sin(DYAW) * lx + math.cos(DYAW) * ly
        heading = wrap(DYAW + rng.uniform(-math.radians(90), math.radians(90)))
        nav.goToPose(pose(nav, hx, hy, heading))
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.1)
        reached = nav.getResult() == TaskResult.SUCCEEDED

        goal = DockRobot.Goal()
        goal.use_dock_id, goal.dock_id, goal.navigate_to_staging_pose = True, "home_dock", True
        goal.max_staging_time = 120.0
        t0 = nav.get_clock().now()
        result, status = run_action(nav, dock_client, goal, timeout_s=240)
        dock_s = (nav.get_clock().now() - t0).nanoseconds / 1e9
        ok = bool(result and result.success)
        ex, ey, eyaw = gt_error()
        within = abs(ex) <= TOL_X and abs(ey) <= TOL_Y and abs(eyaw) <= TOL_YAW
        entry = {"i": i, "start_map": [round(hx, 2), round(hy, 2), round(heading, 2)], "start_reached": reached,
                 "action_success": ok, "error_code": int(result.error_code) if result else None,
                 "num_retries": int(result.num_retries) if result else None, "dock_sim_s": round(dock_s, 1),
                 "err_x_m": round(ex, 4), "err_y_m": round(ey, 4), "err_yaw_deg": round(math.degrees(eyaw), 2),
                 "docked": ok and within}
        log.append(entry)
        print(f"[{i + 1}/{args.n}] {args.label}: action={'ok' if ok else status} err x={ex * 100:+.1f} cm "
              f"y={ey * 100:+.1f} cm yaw={math.degrees(eyaw):+.1f} deg -> {'DOCKED' if entry['docked'] else 'miss'} "
              f"({dock_s:.0f} s)", flush=True)

        ug = UndockRobot.Goal()
        ug.dock_type = "simple_charging_dock"
        run_action(nav, undock_client, ug, timeout_s=60)

    docked = [e for e in log if e["docked"]]
    summary = {"label": args.label, "seed": args.seed, "n": args.n, "docked": len(docked),
               "action_success": sum(e["action_success"] for e in log),
               "median_abs_err_y_cm": round(float(np.median([abs(e["err_y_m"]) for e in log])) * 100, 2),
               "median_abs_err_x_cm": round(float(np.median([abs(e["err_x_m"]) for e in log])) * 100, 2),
               "median_abs_err_yaw_deg": round(float(np.median([abs(e["err_yaw_deg"]) for e in log])), 2),
               "tolerance": {"x_m": TOL_X, "y_m": TOL_Y, "yaw_deg": math.degrees(TOL_YAW)}, "trials": log}
    json.dump(summary, open(f"/ws/outputs/dock_trials_{args.label}.json", "w"), indent=2)
    print(f"done {args.label}: docked {len(docked)}/{args.n}, action success {summary['action_success']}/{args.n}, "
          f"median |y| {summary['median_abs_err_y_cm']} cm, |yaw| {summary['median_abs_err_yaw_deg']} deg", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
