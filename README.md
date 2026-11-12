# TurtleBot 4: Mapping, Navigation and Vision-Based Docking in Simulation

**Status: work in progress. No results yet.**

A simulated TurtleBot 4 (ROS 2 Jazzy, Gazebo Harmonic) maps a warehouse with RTAB-Map RGB-D SLAM,
navigates it with Nav2, and docks using a camera-detected AprilTag on its charging dock.

## What the stock packages already provide, and what this project adds

| Already in `turtlebot4_*` / Nav2 (Jazzy) | Added here |
|---|---|
| Nav2 docking server, configured for an *external* dock-pose detection | An AprilTag on the simulated dock |
| A Create 3 dock model in Gazebo | Tag detection from the robot's camera (`apriltag_ros`) |
| The dock's ground-truth pose from the simulator | A node that turns tag detections into the docking server's `detected_dock_pose` |
| | Docking success rate and final alignment error from varied start poses, vision vs ground-truth pose |
| | RTAB-Map mapping of the warehouse world and Nav2 navigation on that map |

## Setup

Runs in Docker on Linux (tested on Ubuntu 26.04 with an Intel iGPU).

```bash
docker build -t tb4-nav-docking:jazzy docker/
scripts/run.sh          # shell in the container, repo mounted at /ws, GUI forwarded
```

## Results

To be filled in from actual runs.
