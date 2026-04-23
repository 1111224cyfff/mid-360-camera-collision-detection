#!/usr/bin/env python3

import argparse
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rospy

from lio_sam.msg import SegmentedObjectStateArray, WarningState


@dataclass
class WarningSample:
    arrival_sec: float
    header_stamp_sec: float
    active_level: int
    desired_level: int
    source: str
    reason: str
    system_ready: bool


class ContinuityMonitor:
    def __init__(self, segment_topic: str, warning_topic: str):
        self._lock = threading.Lock()
        self.segment_arrival_secs: List[float] = []
        self.warning_samples: List[WarningSample] = []
        self.first_data_event = threading.Event()

        self._segment_sub = rospy.Subscriber(segment_topic, SegmentedObjectStateArray, self._segment_cb, queue_size=50)
        self._warning_sub = rospy.Subscriber(warning_topic, WarningState, self._warning_cb, queue_size=100)

    def _segment_cb(self, msg: SegmentedObjectStateArray) -> None:
        arrival_sec = time.monotonic()
        with self._lock:
            self.segment_arrival_secs.append(arrival_sec)
        self.first_data_event.set()

    def _warning_cb(self, msg: WarningState) -> None:
        arrival_sec = time.monotonic()
        sample = WarningSample(
            arrival_sec=arrival_sec,
            header_stamp_sec=to_sec(msg.header.stamp),
            active_level=int(msg.active_level),
            desired_level=int(msg.desired_level),
            source=msg.source,
            reason=msg.reason,
            system_ready=bool(msg.system_ready),
        )
        with self._lock:
            self.warning_samples.append(sample)
        self.first_data_event.set()

    def snapshot(self):
        with self._lock:
            return list(self.segment_arrival_secs), list(self.warning_samples)


def to_sec(stamp: rospy.Time) -> float:
    if stamp and not stamp.is_zero():
        return stamp.to_sec()
    now = rospy.Time.now()
    return now.to_sec() if not now.is_zero() else rospy.get_time()


def summarize_gaps(stamps: List[float]):
    if len(stamps) < 2:
        return None
    gaps = [curr - prev for prev, curr in zip(stamps[:-1], stamps[1:])]
    return {
        "count": len(gaps),
        "avg": sum(gaps) / float(len(gaps)),
        "max": max(gaps),
    }


def find_last_segment_before(segment_stamps: List[float], warning_time: float) -> Optional[float]:
    last_stamp = None
    for stamp in segment_stamps:
        if stamp > warning_time:
            break
        last_stamp = stamp
    return last_stamp


def analyze(segment_stamps: List[float], warning_samples: List[WarningSample], gap_threshold: float):
    segment_gap_stats = summarize_gaps(segment_stamps)
    warning_gap_stats = summarize_gaps([sample.arrival_sec for sample in warning_samples])

    warning_during_segment_gaps = []
    for sample in warning_samples:
        last_segment_stamp = find_last_segment_before(segment_stamps, sample.arrival_sec)
        if last_segment_stamp is None:
            continue
        gap_sec = sample.arrival_sec - last_segment_stamp
        if gap_sec > gap_threshold:
            warning_during_segment_gaps.append((gap_sec, sample))

    return {
        "segment_messages": len(segment_stamps),
        "warning_messages": len(warning_samples),
        "segment_gap_stats": segment_gap_stats,
        "warning_gap_stats": warning_gap_stats,
        "warning_during_segment_gaps": warning_during_segment_gaps,
    }


def format_stats(label: str, stats) -> str:
    if not stats:
        return f"{label}: insufficient samples"
    return (
        f"{label}: gap_count={stats['count']} avg_gap={stats['avg']:.3f}s max_gap={stats['max']:.3f}s"
    )


def main():
    parser = argparse.ArgumentParser(description="Check whether /warning/state continues publishing through segmentation dropouts.")
    parser.add_argument("--segment-topic", default="/segment/object_states")
    parser.add_argument("--warning-topic", default="/warning/state")
    parser.add_argument("--duration", type=float, default=30.0, help="Capture duration after the first observed message.")
    parser.add_argument("--wait-timeout", type=float, default=180.0, help="Maximum time to wait for the first message.")
    parser.add_argument("--segment-gap-threshold", type=float, default=0.5, help="Segment silence threshold used to detect held-warning intervals.")
    args = parser.parse_args()

    rospy.init_node("warning_continuity_check", anonymous=True)
    monitor = ContinuityMonitor(args.segment_topic, args.warning_topic)

    rospy.loginfo(
        "[warning_continuity_check] waiting up to %.1fs for first data on %s or %s",
        args.wait_timeout,
        args.segment_topic,
        args.warning_topic,
    )

    if not monitor.first_data_event.wait(args.wait_timeout):
        rospy.logerr("[warning_continuity_check] timed out waiting for test data")
        return 2

    start_wall = time.monotonic()
    rospy.loginfo("[warning_continuity_check] first data observed, capturing for %.1fs", args.duration)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if time.monotonic() - start_wall >= args.duration:
            break
        rate.sleep()

    segment_stamps, warning_samples = monitor.snapshot()
    results = analyze(segment_stamps, warning_samples, args.segment_gap_threshold)

    print("=== warning continuity summary ===")
    print(f"segment_messages={results['segment_messages']}")
    print(f"warning_messages={results['warning_messages']}")
    print(format_stats("segment", results["segment_gap_stats"]))
    print(format_stats("warning", results["warning_gap_stats"]))

    held_samples = results["warning_during_segment_gaps"]
    print(f"warning_messages_after_segment_gap_gt_{args.segment_gap_threshold:.2f}s={len(held_samples)}")
    base_arrival = warning_samples[0].arrival_sec if warning_samples else 0.0
    for gap_sec, sample in held_samples[:8]:
        print(
            "held_warning "
            f"gap_since_segment={gap_sec:.3f}s arrival_dt={sample.arrival_sec - base_arrival:.3f}s "
            f"active={sample.active_level} desired={sample.desired_level} "
            f"source={sample.source} ready={int(sample.system_ready)} reason={sample.reason}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())