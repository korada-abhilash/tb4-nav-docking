#!/usr/bin/env python3
"""Average Gazebo real-time factor over a wall-clock window (run inside the container).

Usage: python3 /ws/scripts/rtf.py [seconds=20]
"""
import re, subprocess, sys, time


def sim_time():
    out = subprocess.run(["gz", "topic", "-e", "-t", "/stats", "-n", "1"], capture_output=True, text=True).stdout
    block = re.search(r"sim_time\s*\{([^}]*)\}", out).group(1)
    sec = re.search(r"(?<!n)sec:\s*(\d+)", block)
    nsec = re.search(r"nsec:\s*(\d+)", block)
    return (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) / 1e9


window = float(sys.argv[1]) if len(sys.argv) > 1 else 20
s0, w0 = sim_time(), time.monotonic()
time.sleep(window)
s1, w1 = sim_time(), time.monotonic()
print(f"RTF {(s1 - s0) / (w1 - w0):.3f} over {w1 - w0:.1f} s wall ({s1 - s0:.1f} s sim)")
