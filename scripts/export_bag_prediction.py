#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import bisect
import csv
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, List, Optional

import rosgraph
import rospy
import rostopic

from lio_sam.msg import WarningState


LEVEL_MAP = {
    WarningState.LEVEL_NONE: "无危险",
    WarningState.LEVEL_NOTICE: "提示",
    WarningState.LEVEL_WARNING: "报警",
    WarningState.LEVEL_EMERGENCY: "紧急",
}


@dataclass
class PredictionSample:
    timestamp_sec: float
    level: str


@dataclass
class ReferenceSample:
    wall_sec: float
    bag_abs_sec: float


def to_sec(stamp: rospy.Time) -> float:
    if stamp and not stamp.is_zero():
        return stamp.to_sec()
    return 0.0


def extract_level(msg: WarningState) -> str:
    level = LEVEL_MAP.get(int(msg.active_level))
    if level is None:
        raise ValueError(f"unsupported warning level: {int(msg.active_level)}")
    return level


class PredictionExporter:
    def __init__(
        self,
        video_name: str,
        output_path: Path,
        warning_topic: str,
        reference_topic: str,
        bag_start_sec: float,
        bag_play_rate: float,
        idle_timeout_sec: float,
        startup_timeout_sec: float,
    ) -> None:
        self.video_name = video_name
        self.output_path = output_path
        self.warning_topic = warning_topic
        self.reference_topic = reference_topic
        self.bag_start_sec = bag_start_sec
        self.bag_play_rate = bag_play_rate
        self.idle_timeout_sec = idle_timeout_sec
        self.startup_timeout_sec = startup_timeout_sec

        self.samples: List[PredictionSample] = []
        self.reference_samples: Deque[ReferenceSample] = deque(maxlen=4096)
        self.last_reference_wall_sec: Optional[float] = None
        self.last_warning_wall_sec: Optional[float] = None
        self.start_wall_sec = time.monotonic()
        self.received_warning = False
        self.received_reference = False
        self.master = rosgraph.Master(rospy.get_name())
        self.resolved_reference_topic = rospy.resolve_name(reference_topic)

        self._reference_sub = self._subscribe_reference_topic(reference_topic)
        self._warning_sub = rospy.Subscriber(warning_topic, WarningState, self._warning_cb, queue_size=200)

    def _subscribe_reference_topic(self, topic_name: str):
        start_wait = time.monotonic()
        while not rospy.is_shutdown():
            topic_class, resolved_topic, _ = rostopic.get_topic_class(topic_name, blocking=False)
            if topic_class is not None:
                rospy.loginfo(
                    "[export_bag_prediction] reference topic %s resolved as %s",
                    resolved_topic,
                    topic_class.__name__,
                )
                self.resolved_reference_topic = resolved_topic
                return rospy.Subscriber(resolved_topic, topic_class, self._reference_cb, queue_size=200)
            if (time.monotonic() - start_wait) >= self.startup_timeout_sec:
                raise RuntimeError(f"timed out waiting for reference topic: {topic_name}")
            time.sleep(0.2)

        raise RuntimeError("ROS shutdown before reference topic became available")

    def _reference_cb(self, msg) -> None:
        header = getattr(msg, "header", None)
        stamp_sec = to_sec(header.stamp) if header is not None else 0.0
        if stamp_sec <= 0.0:
            return

        wall_sec = time.monotonic()
        self.reference_samples.append(ReferenceSample(wall_sec=wall_sec, bag_abs_sec=stamp_sec))
        self.last_reference_wall_sec = wall_sec
        if not self.received_reference:
            rospy.loginfo(
                "[export_bag_prediction] first reference sample: wall=%.6f bag=%.6f",
                wall_sec,
                stamp_sec,
            )
        self.received_reference = True

    def _estimate_bag_abs_sec(self, wall_sec: float) -> Optional[float]:
        if not self.reference_samples:
            return None

        refs = list(self.reference_samples)
        if len(refs) == 1:
            ref = refs[0]
            return ref.bag_abs_sec + (wall_sec - ref.wall_sec) * self.bag_play_rate

        wall_times = [sample.wall_sec for sample in refs]
        idx = bisect.bisect_left(wall_times, wall_sec)

        if idx <= 0:
            left = refs[0]
            return left.bag_abs_sec + (wall_sec - left.wall_sec) * self.bag_play_rate
        if idx >= len(refs):
            right = refs[-1]
            return right.bag_abs_sec + (wall_sec - right.wall_sec) * self.bag_play_rate

        left = refs[idx - 1]
        right = refs[idx]
        wall_span = right.wall_sec - left.wall_sec
        if wall_span <= 1e-6:
            return right.bag_abs_sec

        ratio = (wall_sec - left.wall_sec) / wall_span
        return left.bag_abs_sec + ratio * (right.bag_abs_sec - left.bag_abs_sec)

    def _warning_cb(self, msg: WarningState) -> None:
        wall_sec = time.monotonic()
        bag_abs_sec = self._estimate_bag_abs_sec(wall_sec)
        if bag_abs_sec is None:
            return

        timestamp_sec = max(0.0, bag_abs_sec - self.bag_start_sec)
        level = extract_level(msg)
        self.samples.append(PredictionSample(timestamp_sec=timestamp_sec, level=level))
        self.last_warning_wall_sec = wall_sec
        self.received_warning = True

    def should_abort_startup(self) -> bool:
        if self.received_warning or self.received_reference:
            return False
        return (time.monotonic() - self.start_wall_sec) >= self.startup_timeout_sec

    def _reference_topic_has_publishers(self) -> bool:
        try:
            publishers, _, _ = self.master.getSystemState()
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "[export_bag_prediction] failed to query ROS master while checking topic publishers: %s",
                exc,
            )
            return True

        for topic_name, nodes in publishers:
            if topic_name == self.resolved_reference_topic:
                return bool(nodes)
        return False

    def should_finish(self) -> bool:
        if not self.received_warning or self.last_reference_wall_sec is None:
            return False
        if self._reference_topic_has_publishers():
            return False
        return (time.monotonic() - self.last_reference_wall_sec) >= self.idle_timeout_sec

    def write_csv(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["video_name", "timestamp_sec", "level"])
            for sample in sorted(self.samples, key=lambda item: item.timestamp_sec):
                writer.writerow([self.video_name, f"{sample.timestamp_sec:.3f}", sample.level])

        rospy.loginfo(
            "[export_bag_prediction] wrote %d rows to %s",
            len(self.samples),
            str(self.output_path),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export /warning/state to prediction.csv for static warning evaluation.")
    parser.add_argument("--video-name", required=True, help="video_name to write into the CSV")
    parser.add_argument("--output", required=True, help="output CSV path")
    parser.add_argument("--warning-topic", default="/warning/state", help="warning topic to subscribe")
    parser.add_argument(
        "--reference-topic",
        default="/livox/imu_192_168_1_125",
        help="header-bearing input topic used to map wall time back to original bag time",
    )
    parser.add_argument(
        "--bag-start-sec",
        required=True,
        type=float,
        help="bag start time in absolute seconds from rosbag info",
    )
    parser.add_argument(
        "--bag-play-rate",
        type=float,
        default=0.5,
        help="rosbag play rate used by the pipeline",
    )
    parser.add_argument(
        "--idle-timeout-sec",
        type=float,
        default=5.0,
        help="wall-clock idle timeout after the last reference-topic message before writing the CSV",
    )
    parser.add_argument(
        "--startup-timeout-sec",
        type=float,
        default=90.0,
        help="wall-clock timeout while waiting for the first reference or warning message",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rospy.init_node("export_bag_prediction", anonymous=False)
    exporter = PredictionExporter(
        video_name=args.video_name,
        output_path=Path(args.output),
        warning_topic=args.warning_topic,
        reference_topic=args.reference_topic,
        bag_start_sec=args.bag_start_sec,
        bag_play_rate=args.bag_play_rate,
        idle_timeout_sec=args.idle_timeout_sec,
        startup_timeout_sec=args.startup_timeout_sec,
    )

    try:
        while not rospy.is_shutdown():
            if exporter.should_abort_startup():
                raise RuntimeError("timed out waiting for reference or warning messages")
            if exporter.should_finish():
                break
            time.sleep(0.1)
    finally:
        exporter.write_csv()


if __name__ == "__main__":
    main()