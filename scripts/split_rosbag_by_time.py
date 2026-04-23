#!/usr/bin/env python3
"""Clip an existing ROS bag into one or more timestamp windows.

The source bag is opened read-only. Output bags are written to a separate
directory, so the input file content is never modified.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

import rosbag


# Edit these defaults if you want to run this script without CLI arguments.
DEFAULT_BAG_PATH = "/home/nvidia/ws_livox/data/barrier.bag"
DEFAULT_OUT_DIR = "/home/nvidia/ws_livox/data/barrier_split_output"
DEFAULT_PREFIX = None
DEFAULT_FORCE = False
DEFAULT_SEGMENTS = [
    (1776054397.579, 1776054460.672),
    (1776054460.672, 1776054523.765),
]


def parse_segment_text(segment_text: str) -> Tuple[float, float]:
    parts = segment_text.split(":", 1)
    if len(parts) != 2:
        raise ValueError(
            f"Invalid segment '{segment_text}'. Expected format: START:END"
        )

    start_time = float(parts[0])
    end_time = float(parts[1])
    return start_time, end_time


def resolve_segments(segment_texts: List[str]) -> List[Tuple[float, float]]:
    if segment_texts:
        return [parse_segment_text(segment_text) for segment_text in segment_texts]
    return list(DEFAULT_SEGMENTS)


def validate_segments(segments: List[Tuple[float, float]]) -> None:
    if not segments:
        raise ValueError("At least one segment is required")

    previous_end = None
    for index, (start_time, end_time) in enumerate(segments, start=1):
        if start_time >= end_time:
            raise ValueError(
                f"Segment {index} has invalid range: start={start_time}, end={end_time}"
            )

        if previous_end is not None and start_time < previous_end:
            raise ValueError(
                "Segments must be ordered by time and must not overlap"
            )

        previous_end = end_time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Clip an existing ROS bag into one or more timestamp windows.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "bag",
        nargs="?",
        default=DEFAULT_BAG_PATH,
        help="Path to input .bag file",
    )
    parser.add_argument(
        "--out-dir",
        default=DEFAULT_OUT_DIR,
        help="Output directory (default: <bag_stem>_split next to input bag)",
    )
    parser.add_argument(
        "--prefix",
        default=DEFAULT_PREFIX,
        help="Output bag filename prefix (default: input bag basename)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        default=DEFAULT_FORCE,
        help="Allow writing into a non-empty output directory",
    )
    parser.add_argument(
        "--segment",
        action="append",
        default=None,
        help="Timestamp window in START:END format. Repeat for multiple output bags.",
    )
    return parser.parse_args()


def describe_bag(bag_path: Path) -> Tuple[float, float, int]:
    with rosbag.Bag(str(bag_path), "r") as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        message_count = bag.get_message_count()
    return start_time, end_time, message_count


def main() -> int:
    args = parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_file():
        print(f"[ERROR] Bag file not found: {bag_path}")
        return 1

    try:
        segments = resolve_segments(args.segment)
        validate_segments(segments)
    except ValueError as exc:
        print(f"[ERROR] {exc}")
        return 1

    out_dir = (
        Path(args.out_dir).expanduser().resolve()
        if args.out_dir
        else bag_path.parent / f"{bag_path.stem}_split"
    )
    prefix = args.prefix if args.prefix else bag_path.stem

    out_dir.mkdir(parents=True, exist_ok=True)
    if not args.force and any(out_dir.iterdir()):
        print(f"[ERROR] Output directory is not empty: {out_dir}")
        print("[ERROR] Use --force or choose a different --out-dir")
        return 1

    with rosbag.Bag(str(bag_path), "r") as in_bag:
        bag_start_time = in_bag.get_start_time()
        bag_end_time = in_bag.get_end_time()

        print(f"[INFO] input         : {bag_path}")
        print(f"[INFO] output dir    : {out_dir}")
        print(f"[INFO] bag start     : {bag_start_time:.3f}")
        print(f"[INFO] bag end       : {bag_end_time:.3f}")
        print(f"[INFO] segment count : {len(segments)}")

        for index, (segment_start, segment_end) in enumerate(segments, start=1):
            print(
                f"[INFO] segment {index:02d}    : "
                f"[{segment_start:.3f}, {segment_end:.3f})"
            )

        current_out_bag = None
        current_segment_index = 0
        part_paths: List[Path] = []
        segment_message_counts = [0] * len(segments)

        try:
            for topic, msg, t in in_bag.read_messages():
                msg_time = t.to_sec()

                while current_segment_index < len(segments):
                    _, current_segment_end = segments[current_segment_index]
                    if msg_time < current_segment_end:
                        break

                    if current_out_bag is not None:
                        current_out_bag.close()
                        current_out_bag = None

                    current_segment_index += 1

                if current_segment_index >= len(segments):
                    break

                current_segment_start, current_segment_end = segments[current_segment_index]
                if msg_time < current_segment_start:
                    continue

                if current_out_bag is None:
                    out_path = out_dir / f"{prefix}_part{current_segment_index + 1:02d}.bag"
                    current_out_bag = rosbag.Bag(str(out_path), "w")
                    part_paths.append(out_path)
                    print(
                        f"[INFO] writing {out_path.name}: "
                        f"[{current_segment_start:.3f}, {current_segment_end:.3f})"
                    )

                current_out_bag.write(topic, msg, t)
                segment_message_counts[current_segment_index] += 1
        finally:
            if current_out_bag is not None:
                current_out_bag.close()

    print(f"[DONE] Wrote {len(part_paths)} split bag(s)")
    for index, (segment_start, segment_end) in enumerate(segments, start=1):
        part_path = out_dir / f"{prefix}_part{index:02d}.bag"
        if segment_message_counts[index - 1] == 0:
            print(
                f"[WARN] {part_path.name}: no messages in "
                f"[{segment_start:.3f}, {segment_end:.3f})"
            )
            continue

        part_start, part_end, message_count = describe_bag(part_path)
        print(
            f"[DONE] {part_path.name}: "
            f"duration={part_end - part_start:.2f}s messages={message_count}"
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())