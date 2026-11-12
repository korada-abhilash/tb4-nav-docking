#!/usr/bin/env python3
"""Ground-truth model pose from Gazebo via `gz model -m <name> -p` (x, y, yaw in the world frame).

The Pose_V -> TFMessage bridge drops model names, so the CLI is used; it takes about 1 s, which is fine
for scoring a stationary robot at the end of a trial.
"""
import re
import subprocess


def gt_pose(model="turtlebot4"):
    out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=30).stdout
    nums = re.findall(r"\[([-\d.e ]+)\]", out.split("Pose")[-1])
    xyz, rpy = [list(map(float, n.split())) for n in nums[:2]]
    return xyz[0], xyz[1], rpy[2]


if __name__ == "__main__":
    print(gt_pose())
