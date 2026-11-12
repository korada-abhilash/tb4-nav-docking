#!/usr/bin/env python3
"""Feed Nav2's docking server (SimpleChargingDock, use_external_detection_pose) with the detected dock tag.

For every apriltag_ros detection of tag 0, solves the tag pose from its four corner pixels with
cv2.solvePnP (IPPE_SQUARE) and the camera intrinsics, and publishes it as geometry_msgs/PoseStamped on
/detected_dock_pose (camera optical frame, stamped with the image time). Working from the detection
message itself, rather than looking up apriltag_ros's TF afterwards, means a detection is never dropped
because its transform arrived late; dropped detections trip the docking server's
external_detection_timeout during the final approach.

The docking server turns the tag pose into the dock pose with its external_detection_* offsets, which
calibrate_tag.py measures from this same topic. Tag frame: x right, y up, z out of the tag toward the camera.

Usage (in the container): python3 /ws/scripts/dock_pose_relay.py [tag_size_m=0.20]
"""
import math
import sys

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

TAG_ID = 0


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    return x, y, z, w


def make_T(t, q):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = quat_to_mat(q), t
    return T


def yaw_T(x, y, yaw):
    return make_T((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


class Relay(Node):
    def __init__(self, size):
        super().__init__("dock_pose_relay")
        s = size / 2
        # apriltag corner order (counter-clockwise from bottom-left), which is also IPPE_SQUARE's required order
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
        self.K = self.D = None
        self.frame = None
        self.pub = self.create_publisher(PoseStamped, "/detected_dock_pose", 10)
        self.create_subscription(CameraInfo, "/rgbd_camera/camera_info", self.on_info, 1)
        self.create_subscription(AprilTagDetectionArray, "/detections", self.on_detections, 10)

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)
        self.frame = msg.header.frame_id

    def on_detections(self, msg):
        det = next((d for d in msg.detections if d.id == TAG_ID), None)
        if det is None or self.K is None:
            return
        img = np.array([[c.x, c.y] for c in det.corners], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(self.obj, img, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        R, _ = cv2.Rodrigues(rvec)
        p = PoseStamped()
        p.header.frame_id, p.header.stamp = self.frame, msg.header.stamp
        p.pose.position.x, p.pose.position.y, p.pose.position.z = map(float, tvec.ravel())
        (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
         p.pose.orientation.w) = mat_to_quat(R)
        self.pub.publish(p)


def main():
    rclpy.init()
    node = Relay(float(sys.argv[1]) if len(sys.argv) > 1 else 0.20)
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    rclpy.spin(node)


if __name__ == "__main__":
    main()
