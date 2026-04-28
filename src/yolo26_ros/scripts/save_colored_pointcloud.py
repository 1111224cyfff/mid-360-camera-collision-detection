#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

TARGET_SEC = 1776054436
TARGET_NSEC = 778960652
TARGET_TIME = TARGET_SEC + TARGET_NSEC * 1e-9
TOLERANCE_SEC = 0.05  # 允许 ±50ms 误差
OUTPUT_DIR = "/home/nvidia/ws_livox/data"
SAVED = False


def save_pcd(path, points, fields):
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS {}\n".format(" ".join(fields)))
        f.write("SIZE {}\n".format(" ".join(["4"] * len(fields))))
        f.write("TYPE {}\n".format(" ".join(["F"] * len(fields))))
        f.write("COUNT {}\n".format(" ".join(["1"] * len(fields))))
        f.write("WIDTH {}\n".format(len(points)))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS {}\n".format(len(points)))
        f.write("DATA ascii\n")
        for p in points:
            f.write(" ".join([str(v) for v in p]) + "\n")
    rospy.loginfo("Saved PCD: {} ({} points)".format(path, len(points)))


def cloud_callback(msg):
    global SAVED
    if SAVED:
        return

    stamp = msg.header.stamp
    msg_time = stamp.secs + stamp.nsecs * 1e-9
    diff = abs(msg_time - TARGET_TIME)

    rospy.loginfo_throttle(5.0, "/colored_point_cloud timestamp: {}.{:09d} (diff={:.4f}s)".format(
        stamp.secs, stamp.nsecs, diff))

    if diff > TOLERANCE_SEC:
        return

    SAVED = True
    rospy.loginfo("Matched timestamp: {}.{:09d} (diff={:.6f}s)".format(stamp.secs, stamp.nsecs, diff))

    gen = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = list(gen)

    if not points:
        rospy.logwarn("No points found in the cloud.")
        return

    filename = "colored_point_cloud_{}_{:09d}.pcd".format(stamp.secs, stamp.nsecs)
    filepath = os.path.join(OUTPUT_DIR, filename)
    save_pcd(filepath, points, ["x", "y", "z", "rgb"])

    rospy.signal_shutdown("Done")


if __name__ == "__main__":
    rospy.init_node("save_colored_pointcloud")
    rospy.Subscriber("/colored_point_cloud", PointCloud2, cloud_callback)
    rospy.loginfo("Waiting for /colored_point_cloud near timestamp {}.{:09d} (tolerance={}s)...".format(
        TARGET_SEC, TARGET_NSEC, TOLERANCE_SEC))
    rospy.spin()
