#!/usr/bin/env python3
"""Print the closest lidar returns and their bearings (debug helper)."""
import math
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan

rclpy.init()
n = rclpy.create_node("scan_probe")
msg = []
n.create_subscription(LaserScan, "/scan", lambda m: msg.append(m), 10)
while not msg:
    rclpy.spin_once(n, timeout_sec=1.0)
m = msg[0]
r = np.array(m.ranges)
ang = m.angle_min + np.arange(len(r)) * m.angle_increment
print(f"frame {m.header.frame_id}, range_min {m.range_min}, {len(r)} beams, finite {np.isfinite(r).sum()}")
close = np.where(np.isfinite(r) & (r < 0.5))[0]
print(f"returns closer than 0.5 m: {len(close)}")
for i in close[:: max(1, len(close) // 12)]:
    print(f"  bearing {math.degrees(ang[i]):7.1f} deg  range {r[i]:.3f} m")
