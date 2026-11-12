#!/usr/bin/env python3
"""Check the dock pose that the docking server would compute from the current tag detection.

Applies config/tag_calibration.yaml's external_detection_* offsets to /detected_dock_pose the way
SimpleChargingDock does (pose transformed to the fixed frame, orientation = tag * R_ext with only yaw kept,
position = tag + R(yaw) * (tx, ty)), then compares with the true dock pose (Gazebo ground truth, mapped into
odom through the robot's true pose). Also prints the R_ext * tag ordering for comparison.

Usage (in the container, robot stationary with the tag in view, relay running): python3 check_dock_estimate.py
"""
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "/ws/scripts")
from dock_pose_relay import make_T, yaw_T  # noqa: E402
from gt_pose import gt_pose  # noqa: E402


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


cal = yaml.safe_load(Path("/ws/config/tag_calibration.yaml").read_text())["docking_server"]
R_ext = rpy_to_R(cal["external_detection_rotation_roll"], cal["external_detection_rotation_pitch"],
                 cal["external_detection_rotation_yaw"])
tx, ty = cal["external_detection_translation_x"], cal["external_detection_translation_y"]
env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
sx, sy, syaw = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])
dx, dy, dyaw = float(env["DOCK_MAP_X"]), float(env["DOCK_MAP_Y"]), float(env["DOCK_MAP_YAW"])
T_world_dock = yaw_T(sx + math.cos(syaw) * dx - math.sin(syaw) * dy, sy + math.sin(syaw) * dx + math.cos(syaw) * dy,
                     syaw + dyaw)

rclpy.init()
node = rclpy.create_node("check_dock_estimate")
node.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
buf = Buffer()
TransformListener(buf, node)
msgs = []
node.create_subscription(PoseStamped, "/detected_dock_pose", msgs.append, 10)
T_odom_tag = None
while T_odom_tag is None:
    rclpy.spin_once(node, timeout_sec=0.5)
    if msgs:
        m = msgs[-1]
        try:
            t = buf.lookup_transform("odom", m.header.frame_id, Time.from_msg(m.header.stamp))
            b = buf.lookup_transform("odom", "base_link", Time.from_msg(m.header.stamp))
        except Exception:  # noqa: BLE001
            continue
        tr, ro = t.transform.translation, t.transform.rotation
        T_odom_cam = make_T((tr.x, tr.y, tr.z), (ro.x, ro.y, ro.z, ro.w))
        po, q = m.pose.position, m.pose.orientation
        T_odom_tag = T_odom_cam @ make_T((po.x, po.y, po.z), (q.x, q.y, q.z, q.w))
        bt, br = b.transform.translation, b.transform.rotation
        T_odom_base = make_T((bt.x, bt.y, bt.z), (br.x, br.y, br.z, br.w))

gx, gy, gyaw = gt_pose()
T_odom_world = T_odom_base @ np.linalg.inv(yaw_T(gx, gy, gyaw))  # odom pose of the true robot = its odom estimate
true = T_odom_world @ T_world_dock
true_yaw = math.atan2(true[1, 0], true[0, 0])
for name, R in (("tag*R_ext (server)", T_odom_tag[:3, :3] @ R_ext), ("R_ext*tag", R_ext @ T_odom_tag[:3, :3])):
    yaw = math.atan2(R[1, 0], R[0, 0])
    x = T_odom_tag[0, 3] + math.cos(yaw) * tx - math.sin(yaw) * ty
    y = T_odom_tag[1, 3] + math.sin(yaw) * tx + math.cos(yaw) * ty
    print(f"{name:20s} dock est (odom) x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} | true x={true[0, 3]:.3f} "
          f"y={true[1, 3]:.3f} yaw={math.degrees(true_yaw):.1f}")
