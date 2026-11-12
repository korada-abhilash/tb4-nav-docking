#!/usr/bin/env python3
"""Drive the mapping tour: send each waypoint to Nav2 (running alongside RTAB-Map's live map).

Unreachable or timed-out waypoints are skipped and logged. Progress goes to stdout and a JSON summary
to /ws/outputs/map_tour.json.

Usage (in the container, with sim + RTAB-Map + Nav2 running):
    python3 /ws/scripts/map_tour.py [/ws/config/mapping_waypoints.yaml] [goal_timeout_sim_s=120]
"""
import json
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

WAYPOINTS = sys.argv[1] if len(sys.argv) > 1 else "/ws/config/mapping_waypoints.yaml"
GOAL_TIMEOUT = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0


def pose(nav, x, y, yaw=0.0):
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

    # SLAM mode: no AMCL / map_server to wait for. Wait on the action server itself; polling the
    # bt_navigator lifecycle state service timed out repeatedly in this (slow, RTF ~0.3) sim.
    nav.nav_to_pose_client.wait_for_server()
    pts = yaml.safe_load(open(WAYPOINTS))["waypoints"]
    log, t_start = [], nav.get_clock().now()
    for i, (x, y) in enumerate(pts):
        prev = pts[i - 1] if i else (0.0, 0.0)
        yaw = math.atan2(y - prev[1], x - prev[0])
        nav.goToPose(pose(nav, x, y, yaw))
        t0 = nav.get_clock().now()
        while not nav.isTaskComplete():
            if (nav.get_clock().now() - t0).nanoseconds / 1e9 > GOAL_TIMEOUT:
                nav.cancelTask()
                break
            time.sleep(0.2)
        result = nav.getResult()
        sim_s = (nav.get_clock().now() - t0).nanoseconds / 1e9
        log.append({"i": i, "x": x, "y": y, "result": str(result).split(".")[-1], "sim_s": round(sim_s, 1)})
        print(f"[{i + 1}/{len(pts)}] ({x:.1f}, {y:.1f}) {log[-1]['result']} in {sim_s:.0f} s sim", flush=True)

    total = (nav.get_clock().now() - t_start).nanoseconds / 1e9
    ok = sum(e["result"] == "SUCCEEDED" for e in log)
    summary = {"waypoints": len(log), "succeeded": ok, "total_sim_s": round(total, 1), "goals": log}
    json.dump(summary, open("/ws/outputs/map_tour.json", "w"), indent=2)
    print(f"done: {ok}/{len(log)} waypoints reached, {total / 60:.1f} min sim", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
