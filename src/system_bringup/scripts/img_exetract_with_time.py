import argparse
from datetime import datetime
import os

import cv2
import numpy as np
import rosbag
 
def _decode_compressed_image(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("cv2.imdecode returned None; message data may be invalid")
    return image


def extract_images_from_bag(bag_file, output_dir, topic='/hikrobot_camera/rgb/compressed', limit=None):
    os.makedirs(output_dir, exist_ok=True)

    saved = 0
    with rosbag.Bag(bag_file, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            timestamp = t.to_sec()
            time_str = datetime.fromtimestamp(timestamp).strftime('%Y%m%d_%H%M%S_%f')

            # Most bags publish /rgb/compressed as sensor_msgs/CompressedImage.
            if getattr(msg, '_type', None) == 'sensor_msgs/CompressedImage' or hasattr(msg, 'data'):
                cv_image = _decode_compressed_image(msg)
            else:
                raise TypeError(f"Unsupported message type on {topic}: {getattr(msg, '_type', type(msg))}")

            file_path = os.path.join(output_dir, f"{time_str}.png")
            ok = cv2.imwrite(file_path, cv_image)
            if not ok:
                raise IOError(f"Failed to write image: {file_path}")

            saved += 1
            if limit is not None and saved >= limit:
                break
 
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract images from a ROS bag (CompressedImage).')
    parser.add_argument('--bag', default='/home/nvidia/ws_livox/data/20260205/car-dynamic-hook-dynamic-1.bag')
    parser.add_argument('--out', default='/home/nvidia/ws_livox/data/20260205/images')
    parser.add_argument('--topic', default='/hikrobot_camera/rgb/compressed')
    parser.add_argument('--limit', type=int, default=None, help='Max number of images to save')
    args = parser.parse_args()

    extract_images_from_bag(args.bag, args.out, topic=args.topic, limit=args.limit)