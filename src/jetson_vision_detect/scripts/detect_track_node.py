#!/usr/bin/env python3

import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

from jetson_vision_detect.msg import TrackedDetection2D, TrackedDetections2D


def _decode_compressed_image(msg: CompressedImage) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("cv2.imdecode returned None; message data may be invalid")
    return image


def _to_cuda_rgba(bgr: np.ndarray):
    rgba = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGBA)
    rgba = np.ascontiguousarray(rgba)
    return rgba


class JetsonDetectTrackNode:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/hikrobot_camera/rgb/compressed")
        self.use_compressed = rospy.get_param("~use_compressed", True)

        self.model = rospy.get_param("~model", "ssd-mobilenet-v2")
        self.threshold = float(rospy.get_param("~threshold", 0.5))

        self.enable_tracking = rospy.get_param("~enable_tracking", True)
        self.tracker_min_frames = int(rospy.get_param("~tracker_min_frames", 3))
        self.tracker_drop_frames = int(rospy.get_param("~tracker_drop_frames", 15))
        self.tracker_overlap = float(rospy.get_param("~tracker_overlap", 0.5))

        self.publish_overlay = rospy.get_param("~publish_overlay", True)
        self.overlay_topic = rospy.get_param("~overlay_topic", "~overlay")
        self.detections_topic = rospy.get_param("~detections_topic", "~detections")

        self._last_log_time = 0.0
        self._frame_count = 0
        self._t0 = time.time()

        try:
            import jetson.inference  # type: ignore
            import jetson.utils  # type: ignore

            self.jetson_inference = jetson.inference
            self.jetson_utils = jetson.utils
        except Exception as e:
            rospy.logerr("jetson-inference Python module not found (import jetson.* failed): %s", e)
            rospy.logerr("Install/build jetson-inference on the host, or run ros_deep_learning in a container.")
            raise

        argv = []
        self.net = self.jetson_inference.detectNet(self.model, argv=argv, threshold=self.threshold)

        if self.enable_tracking:
            try:
                self.net.SetTrackingEnabled(True)
                self.net.SetTrackingParams(
                    minFrames=self.tracker_min_frames,
                    dropFrames=self.tracker_drop_frames,
                    overlapThreshold=self.tracker_overlap,
                )
                rospy.loginfo(
                    "Tracking enabled (minFrames=%d dropFrames=%d overlap=%.2f)",
                    self.tracker_min_frames,
                    self.tracker_drop_frames,
                    self.tracker_overlap,
                )
            except Exception as e:
                rospy.logwarn("Failed enabling tracking, continuing without it: %s", e)

        self.pub_det = rospy.Publisher(self.detections_topic, TrackedDetections2D, queue_size=5)
        self.pub_overlay = rospy.Publisher(self.overlay_topic, Image, queue_size=2) if self.publish_overlay else None

        if self.use_compressed:
            self.sub = rospy.Subscriber(self.image_topic, CompressedImage, self._cb_compressed, queue_size=2)
            rospy.loginfo("Subscribing to CompressedImage %s", self.image_topic)
        else:
            self.sub = rospy.Subscriber(self.image_topic, Image, self._cb_raw, queue_size=2)
            rospy.loginfo("Subscribing to Image %s", self.image_topic)

        rospy.loginfo("jetson_vision_detect model=%s threshold=%.2f", self.model, self.threshold)

    def _cb_compressed(self, msg: CompressedImage):
        try:
            bgr = _decode_compressed_image(msg)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Failed decoding compressed image: %s", e)
            return
        self._process_frame(bgr, msg.header)

    def _cb_raw(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", e)
            return
        self._process_frame(bgr, msg.header)

    def _get_class_name(self, class_id: int) -> str:
        try:
            return str(self.net.GetClassDesc(class_id))
        except Exception:
            return ""

    def _process_frame(self, bgr: np.ndarray, header):
        # detectNet expects CUDA RGBA images
        rgba = _to_cuda_rgba(bgr)
        try:
            cuda_img = self.jetson_utils.cudaFromNumpy(rgba)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "cudaFromNumpy failed: %s", e)
            return

        try:
            detections = self.net.Detect(cuda_img, overlay="none")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "detectNet.Detect failed: %s", e)
            return

        out = TrackedDetections2D()
        out.header = header

        overlay_img = bgr.copy() if self.pub_overlay is not None else None

        for det in detections:
            td = TrackedDetection2D()
            td.class_id = int(det.ClassID)
            td.class_name = self._get_class_name(td.class_id)
            td.confidence = float(det.Confidence)

            td.left = float(det.Left)
            td.top = float(det.Top)
            td.right = float(det.Right)
            td.bottom = float(det.Bottom)

            td.track_id = int(getattr(det, "TrackID", -1))
            td.track_status = int(getattr(det, "TrackStatus", 0))
            td.track_frames = int(getattr(det, "TrackFrames", 0))
            td.track_lost = int(getattr(det, "TrackLost", 0))

            out.detections.append(td)

            if overlay_img is not None:
                p1 = (int(td.left), int(td.top))
                p2 = (int(td.right), int(td.bottom))
                color = (0, 255, 0) if td.track_status >= 1 else (0, 200, 255)
                cv2.rectangle(overlay_img, p1, p2, color, 2)
                label = f"{td.class_name or td.class_id}:{td.confidence:.2f}"
                if td.track_id >= 0:
                    label += f" id={td.track_id}"
                cv2.putText(overlay_img, label, (p1[0], max(0, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        self.pub_det.publish(out)

        if overlay_img is not None:
            try:
                ros_img = self.bridge.cv2_to_imgmsg(overlay_img, encoding="bgr8")
                ros_img.header = header
                self.pub_overlay.publish(ros_img)
            except Exception as e:
                rospy.logwarn_throttle(2.0, "Failed publishing overlay image: %s", e)

        self._frame_count += 1
        now = time.time()
        if now - self._last_log_time > 2.0:
            dt = now - self._t0
            fps = (self._frame_count / dt) if dt > 0 else 0.0
            rospy.loginfo("vision detections=%d  fps=%.1f", len(out.detections), fps)
            self._last_log_time = now


def main():
    rospy.init_node("jetson_detect_track", anonymous=False)

    try:
        JetsonDetectTrackNode()
    except Exception:
        # Import/build errors already logged.
        return

    rospy.spin()


if __name__ == "__main__":
    main()
