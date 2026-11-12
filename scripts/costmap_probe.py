#!/usr/bin/env python3
"""Print the local costmap around the robot (debug helper). Run in the container with ROS sourced."""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
n = rclpy.create_node("costmap_probe")
msg = []
n.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: msg.append(m), 1)
while not msg:
    rclpy.spin_once(n, timeout_sec=1.0)
m = msg[0]
g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
cy, cx = m.info.height // 2, m.info.width // 2
print(f"local costmap {m.info.width}x{m.info.height} @ {m.info.resolution:.3f} m, frame {m.header.frame_id}")
print("centre 1.2 m window (rows top=+y), values: . <50, o 50-98, X 99-100, ? unknown")
r = int(0.6 / m.info.resolution)
for row in g[cy + r:cy - r - 1:-1, cx - r:cx + r + 1]:
    print("".join("?" if v < 0 else "X" if v >= 99 else "o" if v >= 50 else "." for v in row))
print("cost at robot centre:", g[cy, cx], " max within 0.2 m:", g[cy - 3:cy + 4, cx - 3:cx + 4].max())
