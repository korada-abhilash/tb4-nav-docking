#!/usr/bin/env python3
"""Correct the tag-to-dock calibration by the mean final docking error of a trial set.

AprilTag docking on seed 0 ended with a near-constant offset (spread ~3 mm, ~2 deg) from the true dock pose,
i.e. a calibration bias rather than noise. The docking server drives the robot onto its dock estimate, so
the estimate's *position* is off by the mean position error; this shifts the external_detection_translation
offsets to cancel it: (tx, ty) is applied in the dock-yaw frame, so (tx, ty) <- (tx, ty) - (mean_x_err, mean_y_err).
Heading is not corrected: the server ends docking on position alone, so the final heading error comes from the
controller's curved approach to a laterally offset point, not from the estimate (correcting it as an estimate
error over-rotated the estimate: +23 deg on validation).
The corrected calibration must be validated on different start poses (another seed) than it was fitted on.

Usage (in the container): python3 apply_bias_correction.py outputs/dock_trials_apriltag.json
Keeps the uncorrected values under 'docking_server_uncorrected' in config/tag_calibration.yaml.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    return math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1, min(1, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])


trials = json.load(open(sys.argv[1]))["trials"]
ex = float(np.mean([t["err_x_m"] for t in trials]))
ey = float(np.mean([t["err_y_m"] for t in trials]))
eyaw = math.radians(float(np.mean([t["err_yaw_deg"] for t in trials])))  # reported only

path = Path("/ws/config/tag_calibration.yaml")
cal = yaml.safe_load(path.read_text())
base = cal.get("docking_server_uncorrected", cal["docking_server"])
cal["docking_server_uncorrected"] = base
cal["docking_server"] = dict(base, external_detection_translation_x=round(base["external_detection_translation_x"] - ex, 4),
                             external_detection_translation_y=round(base["external_detection_translation_y"] - ey, 4))
cal["bias_correction"] = {"fitted_on": sys.argv[1], "mean_err_x_m": round(ex, 4), "mean_err_y_m": round(ey, 4),
                          "mean_err_yaw_deg": round(math.degrees(eyaw), 2)}
path.write_text(yaml.safe_dump(cal, sort_keys=False))
print(yaml.safe_dump({k: cal[k] for k in ("bias_correction", "docking_server")}, sort_keys=False))
