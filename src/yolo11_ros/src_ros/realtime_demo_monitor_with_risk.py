#!/usr/bin/env python3
# realtime_demo_monitor_with_risk.py
# ----------------------------------
# Demo：95 % 概率持续输出危险报警，坐标带小幅随机扰动

import argparse
import random
import rospy
from sensor_msgs.msg import Image   # 监听点云改成 PointCloud2

# ==== 可调参数 =====================================================
DANGER_PROB = 0.95                  # ⚠ 每帧 95 % 概率触发危险
BASE_COORD  = (-6.633, 5.447, 48.503)
JITTER      = 0.05                  # 坐标扰动幅度 ±0.05
# ===================================================================

class DemoPrinter:
    def __init__(self):
        self.frame_id = 0

    def _rand_coord(self):
        return tuple(round(c + random.uniform(-JITTER, JITTER), 5)
                     for c in BASE_COORD)

    def __call__(self, msg):
        self.frame_id += 1
        t = msg.header.stamp.to_sec()

        if random.random() < DANGER_PROB:
            x, y, z = self._rand_coord()
            print(f"[{t:.6f}] Frame {self.frame_id:06d} | "
                  f"危险！当前位置存在越界风险，危险机械坐标：({x}  {y}  {z})！")
        else:
            print(f"[{t:.6f}] Frame {self.frame_id:06d} | 安全，未发现越界行为。")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", default="/left_camera/image",
                        help="ROS topic to subscribe")
    args = parser.parse_args()

    rospy.init_node("realtime_demo_monitor_with_risk", anonymous=True)
    rospy.Subscriber(args.topic, Image, DemoPrinter(), queue_size=50)

    rospy.loginfo(f"[DemoMonitor] 已订阅 {args.topic} ，开始打印假监测日志 …")
    rospy.spin()

if __name__ == "__main__":
    main()
