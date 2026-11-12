#!/usr/bin/env python3
"""Measure the apriltag_ros 'dock_tag' frame in the dock model frame from ground truth.

T_dock_tag = inv(T_world_dock) * T_world_base(GT) * T_base_cam(TF) * T_cam_tag(/detected_dock_pose, i.e. the
relay's PnP pose). The robot stands still facing the dock: T_cam_tag is the median of N detections and
T_world_base is one Gazebo query.
From it, derives the docking server's external_detection_* parameters (the transform it applies to a
detected tag pose to get the dock pose). Writes /ws/config/tag_calibration.yaml.

Calibrate with the robot standing at the staging distance (0.7 m) facing the dock. (Calibrating at the
docked pose was tried and made docking fail: the detection was lost on the approach.)
Usage (in the container; sim, dock, apriltag_node and relay running, robot stationary, tag in view):
    python3 /ws/scripts/calibrate_tag.py [n=50]
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
from dock_pose_relay import make_T, mat_to_quat, yaw_T  # noqa: E402
from gt_pose import gt_pose  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 50
env = dict(l.split("=", 1) for l in Path("/ws/config/sim.env").read_text().splitlines() if "=" in l and not l.startswith("#"))
sx, sy, syaw = float(env["SPAWN_X"]), float(env["SPAWN_Y"]), float(env["SPAWN_YAW"])
dx, dy, dyaw = float(env["DOCK_MAP_X"]), float(env["DOCK_MAP_Y"]), float(env["DOCK_MAP_YAW"])
T_world_dock = yaw_T(sx + math.cos(syaw) * dx - math.sin(syaw) * dy,
                     sy + math.sin(syaw) * dx + math.cos(syaw) * dy, syaw + dyaw)


def tf_to_T(t):
    tr, ro = t.translation, t.rotation
    return make_T((tr.x, tr.y, tr.z), (ro.x, ro.y, ro.z, ro.w))


def rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1, min(1, R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


rclpy.init()
node = rclpy.create_node("calibrate_tag")
node.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
buf = Buffer()
TransformListener(buf, node)
samples = []


def on_pose(msg):
    try:
        base_cam = buf.lookup_transform("base_link", msg.header.frame_id, Time())
    except Exception:  # noqa: BLE001
        return
    po, q = msg.pose.position, msg.pose.orientation
    samples.append((tf_to_T(base_cam.transform), make_T((po.x, po.y, po.z), (q.x, q.y, q.z, q.w))))


node.create_subscription(PoseStamped, "/detected_dock_pose", on_pose, 10)
while len(samples) < N:
    rclpy.spin_once(node, timeout_sec=0.5)

gx, gy, gyaw = gt_pose()
T_world_base = yaw_T(gx, gy, gyaw)
Ts = [np.linalg.inv(T_world_dock) @ T_world_base @ bc @ ct for bc, ct in samples]
t = np.median([T[:3, 3] for T in Ts], axis=0)
R = Ts[len(Ts) // 2][:3, :3]
spread = np.std([T[:3, 3] for T in Ts], axis=0)
q = mat_to_quat(R)
# Nav2 SimpleChargingDock with an external detection: dock orientation = tag orientation * R_ext (then only
# yaw is kept), and dock position = tag position + R(dock yaw) * (tx, ty). So R_ext = inv(R_tag_in_dock) and
# (tx, ty) = -(tag position in the dock frame), x/y only.
R_ext = R.T
r, p, y = rpy(R_ext)
cal = {
    "tag_in_dock": {"xyz": [round(float(v), 4) for v in t], "quat_xyzw": [round(float(v), 5) for v in q],
                    "translation_std_m": [round(float(v), 4) for v in spread], "n": len(samples),
                    "robot_gt_world": [round(gx, 4), round(gy, 4), round(gyaw, 4)]},
    "docking_server": {"external_detection_translation_x": round(float(-t[0]), 4),
                       "external_detection_translation_y": round(float(-t[1]), 4),
                       "external_detection_rotation_roll": round(r, 4),
                       "external_detection_rotation_pitch": round(p, 4),
                       "external_detection_rotation_yaw": round(y, 4)},
}
Path("/ws/config/tag_calibration.yaml").write_text(yaml.safe_dump(cal, sort_keys=False))
print(yaml.safe_dump(cal, sort_keys=False))
