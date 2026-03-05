#!/usr/bin/env python3
"""
pc2_to_custombag.py — Convert PointCloud2 messages in a rosbag
to Livox CustomMsg and write to a new bag, while preserving the original
PointCloud2 messages.

Usage:
python3 pc2_to_custombag.py in.bag out_custom.bag /points_topic [time_field]

• /points_topic —— PointCloud2 所在的 topic（必填）
• time_field —— 点云里时间戳字段名，默认为 'timestamp'
若你的字段叫 'time' / 'gps_time'，在最后加上即可
"""

import os, sys, time
import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg, CustomPoint


# ---------------------------------------------------------------------------
def convert_cloud(cloud: PointCloud2, time_field: str) -> CustomMsg:
    """
    将单帧 PointCloud2 → Livox CustomMsg
    """
    # ====== 绝对基准时间（ns） ======
    base_ns = cloud.header.stamp.secs * 1_000_000_000 + cloud.header.stamp.nsecs
    base_sec = cloud.header.stamp.to_sec()

    pts: list[CustomPoint] = []

    for p in pc2.read_points(
                cloud,
                field_names=['x', 'y', 'z', 'intensity', 'tag', 'line', time_field],
                skip_nans=True):

        x, y, z, intensity, tag, line, ts = p # ts 是 float(sec)·μs封装 → 秒
        offset_ns = int((ts - base_sec) * 1e9) # 相对帧首，ns

        # ---------- 填充 CustomPoint ----------
        pt = CustomPoint()
        pt.offset_time = max(0, min(offset_ns, 0xFFFFFFFF)) # clamp 到 uint32
        pt.x, pt.y, pt.z = x, y, z
        pt.reflectivity = int(max(0, min(255, intensity)))
        pt.tag, pt.line = int(tag), int(line)
        pts.append(pt)

    # ---------- 组装 CustomMsg ----------
    out = CustomMsg()
    out.header = cloud.header
    out.timebase = base_ns # uint64 OK
    out.point_num = len(pts)
    out.lidar_id = 0 # 单雷达 = 0
    out.points = pts
    return out


# --------------------------------------------------------------------

def main(in_bag: str, out_bag: str, topic_in: str, time_field: str):
    with rosbag.Bag(in_bag) as bag_in, rosbag.Bag(out_bag, 'w') as bag_out:
        total = bag_in.get_message_count()
        done = 0
        pc2_n = 0

        t0 = time.time()
        for topic, msg, t in bag_in.read_messages():
            done += 1

            if topic == topic_in and msg._type == 'sensor_msgs/PointCloud2':
                pc2_n += 1
                if pc2_n == 1: # 首帧打印基本信息
                    print(f"\n[Frame 1] points={msg.width*msg.height} frame_id={msg.header.frame_id}")

                new_msg = convert_cloud(msg, time_field)
                bag_out.write('/livox/lidar', new_msg, t)
                bag_out.write(topic, msg, t)  # 保留原始的PointCloud2消息
            else:
                bag_out.write(topic, msg, t)

            # 进度条
            if done % 200 == 0 or done == total:
                pct = done / total * 100
                print(f"\rProgress: {done}/{total} ({pct:.1f}%)", end='')

        dt = time.time() - t0
        print(f"\n\nConversion finished: {pc2_n} PointCloud2 → CustomMsg, "
            f"both original PointCloud2 and CustomMsg topics preserved, "
            f"time cost {dt:.1f}s")


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)

    in_bag, out_bag, topic_in = sys.argv[1:4]
    time_field = sys.argv[4] if len(sys.argv) > 4 else 'timestamp'

    if not os.path.exists(in_bag):
        sys.exit(f"[ERROR] input bag not found: {in_bag}")

    if os.path.exists(out_bag):
        ans = input(f"[WARN] {out_bag} exists — overwrite? (y/n): ")
        if ans.lower() != 'y':
            sys.exit("Abort.")

    main(in_bag, out_bag, topic_in, time_field)