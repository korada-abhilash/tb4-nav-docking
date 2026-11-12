#!/usr/bin/env python3
"""Summarize one RGB-D depth frame and one RGB frame (debug helper)."""
import numpy as np
import rclpy
from sensor_msgs.msg import Image

rclpy.init()
n = rclpy.create_node("depth_probe")
got = {}
for t in ("/rgbd_camera/depth_image", "/rgbd_camera/image"):
    n.create_subscription(Image, t, lambda m, t=t: got.setdefault(t, m), 1)
while len(got) < 2:
    rclpy.spin_once(n, timeout_sec=1.0)
d = got["/rgbd_camera/depth_image"]
dt = np.float32 if d.encoding == "32FC1" else np.uint16
a = np.frombuffer(bytes(d.data), dtype=dt).reshape(d.height, d.width).astype(np.float32)
fin = a[np.isfinite(a)]
print(f"depth {d.width}x{d.height} {d.encoding}: finite {fin.size}/{a.size}, min {fin.min():.3f} median {np.median(fin):.3f} max {fin.max():.3f}")
c = got["/rgbd_camera/image"]
img = np.frombuffer(bytes(c.data), dtype=np.uint8)
print(f"rgb {c.width}x{c.height} {c.encoding}: mean {img.mean():.1f} std {img.std():.1f}")
