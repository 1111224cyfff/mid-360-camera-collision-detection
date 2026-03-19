#!/usr/bin/env python3
# realtime_demo_monitor.py  —— 最简版：必定打印

import argparse
import rospy
from sensor_msgs.msg import Image   # 点云请换 sensor_msgs.msg import PointCloud2

class FramePrinter:
    def __init__(self):
        self.frame_id = 0

    def __call__(self, msg):
        self.frame_id += 1
        bag_time = msg.header.stamp.to_sec()
        now_time = rospy.Time.now().to_sec()
        print(f"[{now_time:.3f}] Frame {self.frame_id:06d} | "
              f"BagTime {bag_time:.6f} | 安全，未发现越界行为。")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", default="/left_camera/image",
                        help="ROS topic to subscribe")
    args = parser.parse_args()

    rospy.init_node("realtime_demo_monitor", anonymous=True)

    rospy.Subscriber(args.topic, Image, FramePrinter(), queue_size=50)
    rospy.loginfo(f"[DemoMonitor] 已订阅 {args.topic} ，开始打印假检测日志 …")
    rospy.spin()

if __name__ == "__main__":
    main()
