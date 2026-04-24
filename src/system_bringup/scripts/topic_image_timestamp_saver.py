#!/usr/bin/env python3
import argparse
import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


class TopicImageTimestampSaver:
    def __init__(self, topic, output_dir, prefix, interval_sec):
        self.topic = topic
        self.output_dir = output_dir
        self.prefix = prefix
        self.interval_sec = max(0.0, float(interval_sec))
        self.last_saved_stamp = None
        self.saved_count = 0

        os.makedirs(self.output_dir, exist_ok=True)

        self.sub_raw = rospy.Subscriber(self.topic, Image, self.raw_cb, queue_size=10)

        rospy.loginfo("[topic_image_timestamp_saver] topic=%s output_dir=%s prefix=%s interval=%.3fs",
                      self.topic, self.output_dir, self.prefix, self.interval_sec)

    def should_save(self, stamp_sec):
        if self.last_saved_stamp is None:
            return True
        return (stamp_sec - self.last_saved_stamp) >= self.interval_sec

    @staticmethod
    def msg_stamp(msg):
        if hasattr(msg, "header") and msg.header.stamp and msg.header.stamp.to_sec() > 0:
            return msg.header.stamp
        return rospy.Time.now()

    def save_image(self, image, stamp):
        stamp_sec = stamp.to_sec()
        if not self.should_save(stamp_sec):
            return

        sec = stamp.secs
        nsec = stamp.nsecs
        filename = f"{sec}_{nsec:09d}_{self.prefix}_{self.saved_count:06d}.jpg"
        out_path = os.path.join(self.output_dir, filename)

        if cv2.imwrite(out_path, image):
            self.last_saved_stamp = stamp_sec
            self.saved_count += 1
            if self.saved_count % 20 == 0:
                rospy.loginfo("[topic_image_timestamp_saver] saved=%d latest=%s", self.saved_count, out_path)
        else:
            rospy.logwarn("[topic_image_timestamp_saver] failed to write image: %s", out_path)

    def raw_cb(self, msg):
        encoding = (msg.encoding or "").lower()
        if encoding not in ("bgr8", "rgb8", "mono8"):
            rospy.logwarn_throttle(2.0, "[topic_image_timestamp_saver] unsupported encoding=%s on %s", msg.encoding, self.topic)
            return

        channels = 1 if encoding == "mono8" else 3
        expected_size = int(msg.height * msg.width * channels)
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size < expected_size:
            rospy.logwarn_throttle(2.0, "[topic_image_timestamp_saver] raw image size mismatch on %s", self.topic)
            return

        image = data[:expected_size].reshape((msg.height, msg.width, channels))
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        self.save_image(image, self.msg_stamp(msg))


def parse_args():
    parser = argparse.ArgumentParser(description="Save topic images with per-frame timestamp in filename")
    parser.add_argument("--topic", required=True, help="Input image topic (Image or CompressedImage)")
    parser.add_argument("--output_dir", required=True, help="Output directory")
    parser.add_argument("--prefix", default="image", help="Filename prefix")
    parser.add_argument("--interval", type=float, default=0.1, help="Minimum save interval in seconds")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("topic_image_timestamp_saver", anonymous=True)
    args = parse_args()
    TopicImageTimestampSaver(args.topic, args.output_dir, args.prefix, args.interval)
    rospy.spin()


if __name__ == "__main__":
    main()
