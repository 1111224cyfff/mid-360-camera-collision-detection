#!/usr/bin/env python3
"""
pc2_to_custombag.py -- convert PointCloud2 bag to Livox CustomMsg bag
usage: rosrun your_pkg pc2_to_custombag.py in.bag out_custom.bag /points_raw
"""
import rosbag, rospy, struct
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from livox_ros_driver2.msg import CustomMsg, CustomPoint

def convert_cloud(cloud: PointCloud2) -> CustomMsg:
    base_time = None # ns of first point
    points = []
    point_count = 0
    
    print(f"Converting PointCloud2 with {cloud.width * cloud.height} points, frame_id: {cloud.header.frame_id}")
    
    try:
        for p in pc2.read_points(cloud,
                                field_names=['x','y','z','intensity','tag','line','timestamp'],
                                skip_nans=True):
            x,y,z,intensity,tag,line,ts = p
            ts_ns = int(ts * 1e3) # float64(us) -> int64(ns)
            if base_time is None:
                base_time = ts_ns
                print(f"Base time set to: {base_time} ns")
            
            pt = CustomPoint()
            pt.offset_time = ts_ns - base_time # uint32 自动截断
            pt.x, pt.y, pt.z = x, y, z
            pt.reflectivity = int(max(0,min(255,intensity)))
            pt.tag = int(tag)
            pt.line = int(line)
            points.append(pt)
            point_count += 1
            
            # 每处理1000个点打印一次进度
            if point_count % 1000 == 0:
                print(f"Processed {point_count} points...", end='\r')
    except Exception as e:
        print(f"Error processing point cloud: {e}")
        raise

    msg = CustomMsg()
    msg.header = cloud.header # 直接沿用 frame_id、stamp
    msg.timebase = base_time
    msg.point_num = len(points)
    msg.lidar_id = 0 # 单雷达就填 0
    msg.points = points
    
    print(f"Converted {point_count} points to CustomMsg format")
    
    # 打印一些点的样本数据
    if points:
        print(f"Sample point data - First point: x={points[0].x:.2f}, y={points[0].y:.2f}, z={points[0].z:.2f}, reflectivity={points[0].reflectivity}")
        if len(points) > 1:
            print(f"Sample point data - Last point: x={points[-1].x:.2f}, y={points[-1].y:.2f}, z={points[-1].z:.2f}, reflectivity={points[-1].reflectivity}")
    
    return msg

def main(inbag, outbag, topic_in):
    print(f"\n{'='*50}")
    print(f"Starting conversion from {inbag} to {outbag}")
    print(f"Input topic: {topic_in}")
    print(f"{'='*50}\n")
    
    start_time = time.time()
    total_msgs = 0
    pc2_msgs = 0
    other_msgs = 0
    
    try:
        # 首先计算总消息数以便显示进度
        print("Counting messages in bag file...")
        input_bag = rosbag.Bag(inbag)
        total_msg_count = input_bag.get_message_count()
        print(f"Total messages in bag: {total_msg_count}")
        
        with rosbag.Bag(outbag, 'w') as out:
            for topic, msg, t in input_bag.read_messages():
                total_msgs += 1
                
                # 显示进度
                if total_msgs % 100 == 0 or total_msgs == 1:
                    percent = (total_msgs / total_msg_count) * 100
                    elapsed = time.time() - start_time
                    print(f"Progress: {total_msgs}/{total_msg_count} ({percent:.1f}%) - Elapsed: {elapsed:.1f}s", end='\r')
                
                if topic == topic_in:
                    pc2_msgs += 1
                    print(f"\nProcessing PointCloud2 message #{pc2_msgs} at time {msg.header.stamp.to_sec():.3f}")
                    custom_msg = convert_cloud(msg)
                    out.write('/livox/lidar', custom_msg, t)
                    print(f"Wrote CustomMsg with {custom_msg.point_num} points to output bag")
                else:
                    other_msgs += 1
                    out.write(topic, msg, t)
        
        # 关闭输入bag
        input_bag.close()
        
        # 打印最终统计信息
        elapsed = time.time() - start_time
        print(f"\n\n{'='*50}")
        print(f"Conversion completed in {elapsed:.2f} seconds")
        print(f"Total messages processed: {total_msgs}")
        print(f"PointCloud2 messages converted: {pc2_msgs}")
        print(f"Other messages copied: {other_msgs}")
        print(f"Output bag: {outbag}")
        print(f"{'='*50}")
        
    except Exception as e:
        print(f"\nError during conversion: {e}")
        import traceback
        traceback.print_exc()
        print(f"Conversion failed after processing {total_msgs} messages ({pc2_msgs} point clouds)")

if __name__ == '__main__':
    import sys
    import os
    
    print("\nPointCloud2 to Livox CustomMsg Converter")
    print("---------------------------------------")
    
    # 参数检查
    if len(sys.argv) < 4:
        print('Usage: pc2_to_custombag.py in.bag out.bag_custom /points_topic')
        print('Example: pc2_to_custombag.py data.bag data_custom.bag /points_raw')
        sys.exit(1)
    
    inbag = sys.argv[1]
    outbag = sys.argv[2]
    topic_in = sys.argv[3]
    
    # 检查输入文件是否存在
    if not os.path.exists(inbag):
        print(f"Error: Input bag file '{inbag}' does not exist!")
        sys.exit(1)
    
    # 检查输出文件是否已存在
    if os.path.exists(outbag):
        response = input(f"Output file '{outbag}' already exists. Overwrite? (y/n): ")
        if response.lower() != 'y':
            print("Conversion cancelled.")
            sys.exit(0)
    
    try:
        main(inbag, outbag, topic_in)
    except KeyboardInterrupt:
        print("\nConversion interrupted by user.")
        sys.exit(1)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)