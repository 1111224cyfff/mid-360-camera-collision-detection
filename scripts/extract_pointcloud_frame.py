#!/usr/bin/env python3
"""Extract one or more PointCloud2 frames from a rosbag.

Single-frame mode:
- Provide --topic and --stamp to export the nearest frame for one point-cloud topic.

Batch mode:
- Provide --topics and --sample-count to uniformly sample frames over the reference
  topic duration, then export the nearest frame for every requested topic.
- This is useful for preparing a fixed set of annotation frames covering the whole bag.
"""

import argparse
import csv
import math
from bisect import bisect_left
from pathlib import Path

import rosbag
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract PointCloud2 frame(s) from a rosbag as PCD/CSV."
    )
    parser.add_argument("--bag", required=True, help="Input rosbag path")

    parser.add_argument("--topic", help="Single-frame mode: PointCloud2 topic to extract")
    parser.add_argument(
        "--topics",
        nargs="+",
        help="Batch mode: one or more PointCloud2 topics to export for each sampled frame",
    )
    parser.add_argument(
        "--reference-topic",
        help="Batch mode: topic used to define the uniform time sampling range; defaults to the first topic in --topics",
    )

    parser.add_argument(
        "--stamp",
        help=(
            "Single-frame mode: target timestamp. Accepted forms: sec_nsec or a filename beginning with sec_nsec, "
            "for example 1776054403_378956299_projection_000000.jpg"
        ),
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=0,
        help="Batch mode: uniformly sample this many frames over the reference topic duration",
    )
    parser.add_argument(
        "--max-delta-sec",
        type=float,
        default=0.06,
        help="Maximum allowed absolute time difference between requested and matched frame",
    )
    parser.add_argument(
        "--format",
        choices=("pcd", "csv", "both"),
        default="both",
        help="Output file format",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/nvidia/ws_livox/data/eval_frames",
        help="Directory for exported files",
    )
    parser.add_argument(
        "--basename",
        help="Single-frame mode: optional output basename. Defaults to <matched_stamp>_<topic_name>",
    )
    parser.add_argument(
        "--manifest-csv",
        help="Batch mode: optional CSV manifest path. Defaults to <output-dir>/manifest.csv",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> str:
    if args.max_delta_sec < 0:
        raise ValueError("--max-delta-sec must be >= 0")

    if args.sample_count > 0:
        if not args.topics:
            raise ValueError("Batch mode requires --topics")
        if args.topic or args.stamp:
            raise ValueError("Batch mode does not use --topic or --stamp")
        if args.reference_topic and args.reference_topic not in args.topics:
            raise ValueError("--reference-topic must be one of the topics in --topics")
        return "batch"

    if not args.topic or not args.stamp:
        raise ValueError("Single-frame mode requires both --topic and --stamp")
    if args.topics or args.reference_topic or args.manifest_csv:
        raise ValueError("Single-frame mode does not use --topics, --reference-topic, or --manifest-csv")
    return "single"


def extract_stamp_prefix(value: str) -> tuple:
    name = Path(value).name
    stem = Path(name).stem
    parts = stem.split("_")
    if len(parts) < 2:
        raise ValueError("Timestamp must contain at least sec_nsec")
    try:
        sec = int(parts[0])
        nsec = int(parts[1])
    except ValueError as exc:
        raise ValueError("Timestamp must begin with integer sec_nsec") from exc
    if nsec < 0 or nsec >= 1_000_000_000:
        raise ValueError("Nanoseconds part must be within [0, 1e9)")
    return sec, nsec


def stamp_to_sec(sec: int, nsec: int) -> float:
    return sec + nsec * 1e-9


def sec_to_stamp_text(value: float) -> str:
    sec = int(math.floor(value))
    nsec = int(round((value - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    return f"{sec}_{nsec:09d}"


def message_stamp(msg, bag_time):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None and (stamp.secs != 0 or stamp.nsecs != 0):
        return stamp
    return bag_time


def stamp_key(stamp) -> str:
    return f"{stamp.secs}_{stamp.nsecs:09d}"


def topic_slug(topic: str) -> str:
    return topic.strip("/").replace("/", "_") or "pointcloud"


def point_field_type(field: PointField) -> tuple:
    if field.datatype == PointField.INT8:
        return 1, "I"
    if field.datatype == PointField.UINT8:
        return 1, "U"
    if field.datatype == PointField.INT16:
        return 2, "I"
    if field.datatype == PointField.UINT16:
        return 2, "U"
    if field.datatype == PointField.INT32:
        return 4, "I"
    if field.datatype == PointField.UINT32:
        return 4, "U"
    if field.datatype == PointField.FLOAT32:
        return 4, "F"
    if field.datatype == PointField.FLOAT64:
        return 8, "F"
    raise ValueError(f"Unsupported PointField datatype: {field.datatype}")


def supported_fields(msg) -> list:
    fields = []
    for field in msg.fields:
        if field.count != 1:
            continue
        try:
            size, type_char = point_field_type(field)
        except ValueError:
            continue
        fields.append((field.name, size, type_char))
    if not fields:
        raise RuntimeError("No supported scalar fields found in PointCloud2 message")
    return fields


def read_rows(msg, field_names: list) -> list:
    rows = []
    for point in point_cloud2.read_points(msg, field_names=field_names, skip_nans=False):
        row = []
        has_nan = False
        for value in point:
            if isinstance(value, float) and not math.isfinite(value):
                has_nan = True
                break
            row.append(value)
        if has_nan:
            continue
        rows.append(row)
    return rows


def write_csv(path: Path, field_names: list, rows: list) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(field_names)
        writer.writerows(rows)


def format_pcd_value(value) -> str:
    if isinstance(value, float):
        return f"{value:.9g}"
    return str(value)


def write_pcd(path: Path, fields: list, rows: list) -> None:
    field_names = [field[0] for field in fields]
    sizes = [str(field[1]) for field in fields]
    types = [field[2] for field in fields]
    counts = ["1" for _ in fields]
    with path.open("w", encoding="ascii") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
        handle.write("VERSION 0.7\n")
        handle.write(f"FIELDS {' '.join(field_names)}\n")
        handle.write(f"SIZE {' '.join(sizes)}\n")
        handle.write(f"TYPE {' '.join(types)}\n")
        handle.write(f"COUNT {' '.join(counts)}\n")
        handle.write(f"WIDTH {len(rows)}\n")
        handle.write("HEIGHT 1\n")
        handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(rows)}\n")
        handle.write("DATA ascii\n")
        for row in rows:
            handle.write(" ".join(format_pcd_value(value) for value in row))
            handle.write("\n")


def export_message(msg, output_dir: Path, basename: str, output_format: str) -> dict:
    fields = supported_fields(msg)
    field_names = [field[0] for field in fields]
    rows = read_rows(msg, field_names)

    result = {
        "point_count": len(rows),
        "fields": field_names,
        "csv_path": "",
        "pcd_path": "",
    }

    if output_format in ("csv", "both"):
        csv_path = output_dir / f"{basename}.csv"
        write_csv(csv_path, field_names, rows)
        result["csv_path"] = str(csv_path)

    if output_format in ("pcd", "both"):
        pcd_path = output_dir / f"{basename}.pcd"
        write_pcd(pcd_path, fields, rows)
        result["pcd_path"] = str(pcd_path)

    return result


def collect_topic_entries(bag_path: Path, topics: list) -> dict:
    topic_data = {topic: {"entries": [], "stamp_secs": []} for topic in topics}
    counters = {topic: 0 for topic in topics}

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            stamp = message_stamp(msg, bag_time)
            topic_data[topic]["entries"].append(
                {
                    "index": counters[topic],
                    "stamp_sec": stamp.to_sec(),
                    "stamp_text": stamp_key(stamp),
                }
            )
            topic_data[topic]["stamp_secs"].append(stamp.to_sec())
            counters[topic] += 1

    for topic in topics:
        if not topic_data[topic]["entries"]:
            raise RuntimeError(f"No messages found on topic: {topic}")

    return topic_data


def find_nearest_entry(topic_info: dict, target_sec: float) -> tuple:
    stamp_secs = topic_info["stamp_secs"]
    entries = topic_info["entries"]
    insert_pos = bisect_left(stamp_secs, target_sec)
    candidate_indices = []
    if insert_pos < len(entries):
        candidate_indices.append(insert_pos)
    if insert_pos > 0:
        candidate_indices.append(insert_pos - 1)
    if not candidate_indices:
        raise RuntimeError("No candidate entries found")

    best_index = min(candidate_indices, key=lambda idx: abs(entries[idx]["stamp_sec"] - target_sec))
    entry = entries[best_index]
    delta = abs(entry["stamp_sec"] - target_sec)
    return entry, delta


def find_nearest_unused_reference_index(stamp_secs: list, target_sec: float, used_indices: set) -> int:
    insert_pos = bisect_left(stamp_secs, target_sec)
    left = insert_pos - 1
    right = insert_pos

    while left >= 0 or right < len(stamp_secs):
        left_candidate = None
        right_candidate = None

        if left >= 0 and left not in used_indices:
            left_candidate = (abs(stamp_secs[left] - target_sec), left)
        if right < len(stamp_secs) and right not in used_indices:
            right_candidate = (abs(stamp_secs[right] - target_sec), right)

        if left_candidate and right_candidate:
            return min(left_candidate, right_candidate)[1]
        if left_candidate:
            return left_candidate[1]
        if right_candidate:
            return right_candidate[1]

        left -= 1
        right += 1

    raise RuntimeError("No unused reference frame available for the requested target time")


def choose_uniform_reference_entries(topic_info: dict, sample_count: int) -> list:
    entries = topic_info["entries"]
    stamp_secs = topic_info["stamp_secs"]
    if sample_count > len(entries):
        raise ValueError(
            f"Requested {sample_count} samples, but reference topic only contains {len(entries)} frames"
        )

    if sample_count == 1:
        target_secs = [0.5 * (stamp_secs[0] + stamp_secs[-1])]
    else:
        interval = (stamp_secs[-1] - stamp_secs[0]) / float(sample_count - 1)
        target_secs = [stamp_secs[0] + interval * index for index in range(sample_count)]

    chosen = []
    used_indices = set()
    for sample_index, target_sec in enumerate(target_secs):
        entry_index = find_nearest_unused_reference_index(stamp_secs, target_sec, used_indices)
        used_indices.add(entry_index)
        chosen.append(
            {
                "sample_index": sample_index,
                "requested_target_sec": target_sec,
                "requested_target_stamp": sec_to_stamp_text(target_sec),
                "reference_entry": entries[entry_index],
            }
        )

    chosen.sort(key=lambda item: item["reference_entry"]["stamp_sec"])
    for sample_index, item in enumerate(chosen):
        item["sample_index"] = sample_index
    return chosen


def build_time_window_filtered_topic_info(topic_info: dict, start_sec: float, end_sec: float) -> dict:
    filtered_entries = [
        entry
        for entry in topic_info["entries"]
        if start_sec <= entry["stamp_sec"] <= end_sec
    ]
    if not filtered_entries:
        raise RuntimeError(
            "Reference topic has no frames inside the common overlap window [{:.6f}, {:.6f}]".format(
                start_sec,
                end_sec,
            )
        )
    return {
        "entries": filtered_entries,
        "stamp_secs": [entry["stamp_sec"] for entry in filtered_entries],
    }


def compute_common_overlap_window(topic_data: dict, topics: list) -> tuple:
    overlap_start = max(topic_data[topic]["stamp_secs"][0] for topic in topics)
    overlap_end = min(topic_data[topic]["stamp_secs"][-1] for topic in topics)
    if overlap_end < overlap_start:
        raise RuntimeError("Requested topics do not share an overlapping time window")
    return overlap_start, overlap_end


def build_valid_reference_topic_info(reference_topic_info: dict, topic_data: dict, topics: list, max_delta_sec: float) -> dict:
    valid_entries = []
    for entry in reference_topic_info["entries"]:
        reference_sec = entry["stamp_sec"]
        is_valid = True
        for topic in topics:
            _, delta = find_nearest_entry(topic_data[topic], reference_sec)
            if delta > max_delta_sec:
                is_valid = False
                break
        if is_valid:
            valid_entries.append(entry)

    if not valid_entries:
        raise RuntimeError(
            "No reference frames remain after enforcing --max-delta-sec={} across all topics".format(
                max_delta_sec
            )
        )

    return {
        "entries": valid_entries,
        "stamp_secs": [entry["stamp_sec"] for entry in valid_entries],
    }


def plan_batch_exports(topic_data: dict, topics: list, reference_topic: str, sample_count: int, max_delta_sec: float) -> list:
    overlap_start, overlap_end = compute_common_overlap_window(topic_data, topics)
    filtered_reference_info = build_time_window_filtered_topic_info(
        topic_data[reference_topic],
        overlap_start,
        overlap_end,
    )
    valid_reference_info = build_valid_reference_topic_info(
        filtered_reference_info,
        topic_data,
        topics,
        max_delta_sec,
    )
    reference_samples = choose_uniform_reference_entries(valid_reference_info, sample_count)
    sample_plans = []

    for sample in reference_samples:
        reference_stamp_sec = sample["reference_entry"]["stamp_sec"]
        topic_matches = {}
        for topic in topics:
            entry, delta = find_nearest_entry(topic_data[topic], reference_stamp_sec)
            if delta > max_delta_sec:
                raise RuntimeError(
                    "Sample {} topic {} matched frame delta {:.6f}s exceeds --max-delta-sec={:.6f}s".format(
                        sample["sample_index"], topic, delta, max_delta_sec
                    )
                )
            topic_matches[topic] = {
                "entry": entry,
                "delta_sec": delta,
            }

        sample_plans.append(
            {
                "sample_index": sample["sample_index"],
                "requested_target_sec": sample["requested_target_sec"],
                "requested_target_stamp": sample["requested_target_stamp"],
                "reference_topic": reference_topic,
                "reference_stamp": sample["reference_entry"]["stamp_text"],
                "topic_matches": topic_matches,
            }
        )

    return sample_plans


def build_batch_export_map(sample_plans: list, output_dir: Path, topics: list) -> dict:
    export_map = {topic: {} for topic in topics}
    for sample in sample_plans:
        sample_dir = output_dir / f"sample_{sample['sample_index']:03d}"
        sample_dir.mkdir(parents=True, exist_ok=True)
        sample["sample_dir"] = str(sample_dir)

        for topic in topics:
            match = sample["topic_matches"][topic]
            basename = "sample_{:03d}_{}__{}".format(
                sample["sample_index"],
                topic_slug(topic),
                match["entry"]["stamp_text"],
            )
            export_map[topic][match["entry"]["index"]] = {
                "sample": sample,
                "topic": topic,
                "sample_dir": sample_dir,
                "basename": basename,
            }
            match["basename"] = basename
    return export_map


def execute_batch_exports(bag_path: Path, topics: list, export_map: dict, output_format: str) -> None:
    counters = {topic: 0 for topic in topics}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            topic_index = counters[topic]
            counters[topic] += 1
            task = export_map[topic].get(topic_index)
            if task is None:
                continue

            exported = export_message(msg, task["sample_dir"], task["basename"], output_format)
            match = task["sample"]["topic_matches"][topic]
            match["point_count"] = exported["point_count"]
            match["fields"] = exported["fields"]
            match["csv_path"] = exported["csv_path"]
            match["pcd_path"] = exported["pcd_path"]


def write_manifest_csv(path: Path, sample_plans: list, topics: list) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "sample_index",
                "requested_target_stamp",
                "requested_target_sec",
                "reference_topic",
                "reference_stamp",
                "topic",
                "matched_stamp",
                "delta_sec",
                "point_count",
                "sample_dir",
                "csv_path",
                "pcd_path",
            ]
        )
        for sample in sample_plans:
            for topic in topics:
                match = sample["topic_matches"][topic]
                writer.writerow(
                    [
                        sample["sample_index"],
                        sample["requested_target_stamp"],
                        f"{sample['requested_target_sec']:.9f}",
                        sample["reference_topic"],
                        sample["reference_stamp"],
                        topic,
                        match["entry"]["stamp_text"],
                        f"{match['delta_sec']:.6f}",
                        match.get("point_count", ""),
                        sample["sample_dir"],
                        match.get("csv_path", ""),
                        match.get("pcd_path", ""),
                    ]
                )


def find_nearest_message(bag_path: Path, topic: str, target_sec: float):
    best_msg = None
    best_bag_time = None
    best_delta = None

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            stamp = message_stamp(msg, bag_time)
            delta = abs(stamp.to_sec() - target_sec)
            if best_delta is None or delta < best_delta:
                best_msg = msg
                best_bag_time = bag_time
                best_delta = delta

    if best_msg is None:
        raise RuntimeError(f"No messages found on topic: {topic}")
    return best_msg, best_bag_time, best_delta


def run_single_mode(args: argparse.Namespace, bag_path: Path, output_dir: Path) -> int:
    sec, nsec = extract_stamp_prefix(args.stamp)
    target_sec = stamp_to_sec(sec, nsec)

    msg, bag_time, delta = find_nearest_message(bag_path, args.topic, target_sec)
    if delta > args.max_delta_sec:
        raise RuntimeError(
            f"Nearest point cloud differs by {delta:.6f}s, larger than --max-delta-sec={args.max_delta_sec:.6f}s"
        )

    matched_stamp = message_stamp(msg, bag_time)
    matched_stamp_text = stamp_key(matched_stamp)
    requested_stamp_text = f"{sec}_{nsec:09d}"

    basename = args.basename if args.basename else f"{matched_stamp_text}_{topic_slug(args.topic)}"
    exported = export_message(msg, output_dir, basename, args.format)

    print("[MATCH]")
    print(f"requested_stamp: {requested_stamp_text}")
    print(f"matched_stamp:   {matched_stamp_text}")
    print(f"delta_sec:       {delta:.6f}")
    print(f"topic:           {args.topic}")
    print(f"point_count:     {exported['point_count']}")
    print(f"fields:          {', '.join(exported['fields'])}")
    print("\n[OUTPUT]")
    if exported["csv_path"]:
        print(exported["csv_path"])
    if exported["pcd_path"]:
        print(exported["pcd_path"])

    return 0


def run_batch_mode(args: argparse.Namespace, bag_path: Path, output_dir: Path) -> int:
    reference_topic = args.reference_topic if args.reference_topic else args.topics[0]
    topic_data = collect_topic_entries(bag_path, args.topics)
    sample_plans = plan_batch_exports(
        topic_data,
        args.topics,
        reference_topic,
        args.sample_count,
        args.max_delta_sec,
    )
    export_map = build_batch_export_map(sample_plans, output_dir, args.topics)
    execute_batch_exports(bag_path, args.topics, export_map, args.format)

    manifest_path = Path(args.manifest_csv).expanduser().resolve() if args.manifest_csv else output_dir / "manifest.csv"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    write_manifest_csv(manifest_path, sample_plans, args.topics)

    print("[BATCH]")
    print(f"sample_count:    {len(sample_plans)}")
    print(f"reference_topic: {reference_topic}")
    print(f"output_dir:      {output_dir}")
    print(f"manifest_csv:    {manifest_path}")
    print("\n[SAMPLES]")
    for sample in sample_plans:
        print(
            "sample_{:03d}: requested={} reference={}".format(
                sample["sample_index"],
                sample["requested_target_stamp"],
                sample["reference_stamp"],
            )
        )
        for topic in args.topics:
            match = sample["topic_matches"][topic]
            print(
                "  {} -> matched={} delta={:.6f}s points={}".format(
                    topic,
                    match["entry"]["stamp_text"],
                    match["delta_sec"],
                    match.get("point_count", ""),
                )
            )

    return 0


def main() -> int:
    args = parse_args()
    mode = validate_args(args)

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag file not found: {bag_path}")

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if mode == "single":
        return run_single_mode(args, bag_path, output_dir)
    return run_batch_mode(args, bag_path, output_dir)


if __name__ == "__main__":
    raise SystemExit(main())