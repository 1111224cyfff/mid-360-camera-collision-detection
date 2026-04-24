#!/usr/bin/env python3
"""Evaluate point-level TP/FP/FN metrics from /colored_point_cloud and GT CSV files.

Ground-truth format (rosbag mode):
- A directory containing one CSV file per evaluated frame.
- Each file name must be <sec>_<nsec>.csv, matching the ROS message stamp.
- Each CSV row must provide x,y,z coordinates for one positive point.
- Header rows are allowed; the script will skip a non-numeric first row.

Prediction source (rosbag mode):
- A rosbag containing PointCloud2 messages on /colored_point_cloud.
- Red points (255,0,0) are treated as predicted positives.
- All other points are treated as predicted negatives.

PCD pairs mode:
- --mode pcd_pairs reads prediction PCD and ground-truth PCD/CSV file pairs.
- Prediction PCD must contain colored points; red (255,0,0) = predicted positive.
- Ground-truth PCD/CSV contains only positive points.
"""

import argparse
import csv
import json
import math
import struct
import sys
from collections import Counter
from pathlib import Path

import numpy as np

import rosbag
from sensor_msgs import point_cloud2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate point-selection metrics against per-frame ground-truth point CSV files."
    )
    parser.add_argument(
        "--mode",
        choices=["rosbag", "pcd_pairs"],
        default="rosbag",
        help="Evaluation mode: rosbag (default) or pcd_pairs",
    )
    parser.add_argument("--bag", help="Rosbag containing /colored_point_cloud messages (rosbag mode)")
    parser.add_argument(
        "--ground-truth-dir",
        help="Directory with per-frame CSV files named <sec>_<nsec>.csv (rosbag mode)",
    )
    parser.add_argument(
        "--pairs-list",
        help="Text file listing PCD/CSV pairs, one per line: pred.pcd gt.pcd (or gt.csv). "
             "Used in pcd_pairs mode.",
    )
    parser.add_argument(
        "--topic",
        default="/colored_point_cloud",
        help="PointCloud2 topic carrying the algorithm output (rosbag mode)",
    )
    parser.add_argument(
        "--coordinate-tolerance",
        type=float,
        default=1e-4,
        help="Quantization tolerance in meters for matching predicted and GT points",
    )
    parser.add_argument(
        "--positive-color",
        default="255,0,0",
        help="RGB triplet used to mark predicted positives in the colored cloud",
    )
    parser.add_argument(
        "--output-json",
        help="Optional path to save the evaluation summary as JSON",
    )
    return parser.parse_args()


def parse_positive_color(value: str) -> tuple:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise ValueError("--positive-color must be formatted as R,G,B")
    try:
        rgb = tuple(int(part) for part in parts)
    except ValueError as exc:
        raise ValueError("--positive-color must contain integers") from exc
    if any(channel < 0 or channel > 255 for channel in rgb):
        raise ValueError("--positive-color values must be within [0, 255]")
    return rgb


def stamp_key(stamp) -> str:
    return f"{stamp.secs}_{stamp.nsecs:09d}"


def discover_ground_truth_files(directory: Path) -> dict:
    if not directory.is_dir():
        raise FileNotFoundError(f"Ground-truth directory not found: {directory}")

    mapping = {}
    for path in sorted(directory.iterdir()):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".csv", ".txt"}:
            continue
        mapping[path.stem] = path

    if not mapping:
        raise FileNotFoundError(
            f"No ground-truth CSV/TXT files found in directory: {directory}"
        )
    return mapping


def parse_point_row(row) -> tuple:
    if len(row) < 3:
        raise ValueError("Each GT row must contain at least x,y,z")
    return float(row[0]), float(row[1]), float(row[2])


def load_ground_truth_points(path: Path) -> list:
    points = []
    with path.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for row_index, row in enumerate(reader):
            if not row:
                continue
            if row_index == 0:
                try:
                    points.append(parse_point_row(row))
                except ValueError:
                    continue
            else:
                points.append(parse_point_row(row))
    return points


def parse_pcd_file(path: Path) -> dict:
    """Parse an ASCII or binary PCD file and return field arrays as a dict."""
    with path.open("rb") as fh:
        lines = []
        while True:
            line = fh.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            lines.append(line)
            if line.startswith("DATA"):
                break

        header = {}
        for line in lines:
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                header[parts[0]] = parts[1:]

        fields = header.get("FIELDS", [])
        sizes = [int(s) for s in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(c) for c in header.get("COUNT", [])]
        width = int(header.get("WIDTH", ["0"])[0])
        height = int(header.get("HEIGHT", ["0"])[0])
        num_points = int(header.get("POINTS", [str(width * height)])[0])
        data_format = header.get("DATA", [""])[0]

        if data_format == "ascii":
            data_lines = fh.read().decode("ascii", errors="ignore").strip().split("\n")
            raw = []
            for dl in data_lines:
                dl = dl.strip()
                if not dl:
                    continue
                raw.append([float(v) for v in dl.split()])
            arr = np.array(raw, dtype=np.float64)
        elif data_format == "binary":
            # Compute struct format for one point
            fmt_parts = []
            for t, s in zip(types, sizes):
                if t == "F" and s == 4:
                    fmt_parts.append("f")
                elif t == "F" and s == 8:
                    fmt_parts.append("d")
                elif t == "I" and s == 4:
                    fmt_parts.append("i")
                elif t == "I" and s == 8:
                    fmt_parts.append("q")
                elif t == "U" and s == 1:
                    fmt_parts.append("B")
                elif t == "U" and s == 2:
                    fmt_parts.append("H")
                elif t == "U" and s == 4:
                    fmt_parts.append("I")
                else:
                    raise RuntimeError(f"Unsupported PCD type/size: {t}/{s}")
            fmt = "<" + "".join(fmt_parts)
            point_size = struct.calcsize(fmt)
            raw = []
            for i in range(num_points):
                blob = fh.read(point_size)
                if len(blob) < point_size:
                    break
                raw.append(struct.unpack(fmt, blob))
            arr = np.array(raw, dtype=np.float64)
        else:
            raise RuntimeError(f"Unsupported PCD DATA format: {data_format}")

        result = {}
        col = 0
        for field, count in zip(fields, counts):
            if count == 1:
                result[field] = arr[:, col]
            else:
                result[field] = arr[:, col : col + count]
            col += count
        return result


def load_pcd_points(path: Path) -> list:
    """Load all xyz points from a PCD file."""
    data = parse_pcd_file(path)
    if "x" not in data or "y" not in data or "z" not in data:
        raise RuntimeError(f"PCD file missing x/y/z fields: {path}")
    pts = np.column_stack((data["x"], data["y"], data["z"]))
    return [(float(x), float(y), float(z)) for x, y, z in pts]


def extract_predicted_positive_points_from_pcd(path: Path, positive_color: tuple) -> list:
    """Extract points matching positive_color from a colored PCD file."""
    data = parse_pcd_file(path)
    if "x" not in data or "y" not in data or "z" not in data:
        raise RuntimeError(f"PCD file missing x/y/z fields: {path}")

    xyz = np.column_stack((data["x"], data["y"], data["z"]))

    if "rgb" in data:
        rgb_field = data["rgb"]
        mask = np.zeros(len(xyz), dtype=bool)
        for i, rgbf in enumerate(rgb_field):
            if unpack_rgb_float(float(rgbf)) == positive_color:
                mask[i] = True
        return [(float(x), float(y), float(z)) for x, y, z in xyz[mask]]

    if {"r", "g", "b"}.issubset(data.keys()):
        r = data["r"].astype(np.uint8)
        g = data["g"].astype(np.uint8)
        b = data["b"].astype(np.uint8)
        mask = (r == positive_color[0]) & (g == positive_color[1]) & (b == positive_color[2])
        return [(float(x), float(y), float(z)) for x, y, z in xyz[mask]]

    raise RuntimeError(
        f"PCD file does not contain rgb or r/g/b fields; cannot recover predicted positives: {path}"
    )


def load_ground_truth_pcd(path: Path) -> list:
    """Load ground-truth points from a PCD file (all points are positive)."""
    return load_pcd_points(path)


def unpack_rgb_float(rgb_float: float) -> tuple:
    packed = struct.unpack("I", struct.pack("f", float(rgb_float)))[0]
    return ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)


def extract_predicted_positive_points(msg, positive_color: tuple) -> list:
    field_names = [field.name for field in msg.fields]
    points = []

    if "rgb" in field_names:
        iterator = point_cloud2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        for x, y, z, rgb in iterator:
            if unpack_rgb_float(rgb) == positive_color:
                points.append((float(x), float(y), float(z)))
        return points

    if {"r", "g", "b"}.issubset(field_names):
        iterator = point_cloud2.read_points(msg, field_names=("x", "y", "z", "r", "g", "b"), skip_nans=True)
        for x, y, z, r, g, b in iterator:
            if (int(r), int(g), int(b)) == positive_color:
                points.append((float(x), float(y), float(z)))
        return points

    raise RuntimeError(
        "PointCloud2 message does not contain rgb or r/g/b fields; cannot recover predicted positives"
    )


def quantize_point(point: tuple, tolerance: float) -> tuple:
    return tuple(int(round(coordinate / tolerance)) for coordinate in point)


def build_counter(points: list, tolerance: float) -> Counter:
    counter = Counter()
    for point in points:
        counter[quantize_point(point, tolerance)] += 1
    return counter


def safe_divide(numerator: float, denominator: float) -> float:
    if denominator == 0:
        return 0.0
    return numerator / denominator


def compute_metrics(tp: int, fp: int, fn: int) -> dict:
    precision = safe_divide(tp, tp + fp)
    recall = safe_divide(tp, tp + fn)
    f1 = safe_divide(2 * precision * recall, precision + recall) if precision + recall > 0 else 0.0
    iou = safe_divide(tp, tp + fp + fn)
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "iou": iou,
    }


def evaluate_frame(predicted_points: list, gt_points: list, tolerance: float) -> dict:
    pred_counter = build_counter(predicted_points, tolerance)
    gt_counter = build_counter(gt_points, tolerance)
    intersection = pred_counter & gt_counter
    tp = sum(intersection.values())
    fp = sum(pred_counter.values()) - tp
    fn = sum(gt_counter.values()) - tp
    metrics = compute_metrics(tp, fp, fn)
    metrics["predicted_positive_count"] = sum(pred_counter.values())
    metrics["ground_truth_positive_count"] = sum(gt_counter.values())
    return metrics


def summarize_frames(frame_results: list) -> dict:
    total_tp = sum(frame["tp"] for frame in frame_results)
    total_fp = sum(frame["fp"] for frame in frame_results)
    total_fn = sum(frame["fn"] for frame in frame_results)
    micro = compute_metrics(total_tp, total_fp, total_fn)

    metric_names = ("precision", "recall", "f1", "iou")
    macro = {
        name: safe_divide(sum(frame[name] for frame in frame_results), len(frame_results))
        for name in metric_names
    }

    micro["frames_evaluated"] = len(frame_results)
    micro["macro_precision"] = macro["precision"]
    micro["macro_recall"] = macro["recall"]
    micro["macro_f1"] = macro["f1"]
    micro["macro_iou"] = macro["iou"]
    return micro


def print_summary(summary: dict, frame_results: list) -> None:
    print("[SUMMARY]")
    print(f"frames_evaluated: {summary['frames_evaluated']}")
    print(f"tp: {summary['tp']}")
    print(f"fp: {summary['fp']}")
    print(f"fn: {summary['fn']}")
    print(f"precision: {summary['precision']:.6f}")
    print(f"recall: {summary['recall']:.6f}")
    print(f"f1: {summary['f1']:.6f}")
    print(f"iou: {summary['iou']:.6f}")
    print(f"macro_precision: {summary['macro_precision']:.6f}")
    print(f"macro_recall: {summary['macro_recall']:.6f}")
    print(f"macro_f1: {summary['macro_f1']:.6f}")
    print(f"macro_iou: {summary['macro_iou']:.6f}")

    print("\n[FRAMES]")
    for frame in frame_results:
        print(
            "{stamp}: tp={tp} fp={fp} fn={fn} precision={precision:.6f} recall={recall:.6f} f1={f1:.6f} iou={iou:.6f}".format(
                **frame
            )
        )


def load_ground_truth_generic(path: Path) -> list:
    """Load ground-truth points from CSV or PCD."""
    suffix = path.suffix.lower()
    if suffix == ".csv" or suffix == ".txt":
        return load_ground_truth_points(path)
    elif suffix == ".pcd":
        return load_ground_truth_pcd(path)
    else:
        raise ValueError(f"Unsupported ground-truth file format: {path}")


def evaluate_rosbag_mode(args, positive_color: tuple) -> tuple:
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag file not found: {bag_path}")

    gt_dir = Path(args.ground_truth_dir).expanduser().resolve()
    gt_files = discover_ground_truth_files(gt_dir)

    frame_results = []
    matched_stamps = set()

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[args.topic]):
            frame_stamp = stamp_key(msg.header.stamp)
            gt_path = gt_files.get(frame_stamp)
            if gt_path is None:
                continue

            gt_points = load_ground_truth_points(gt_path)
            predicted_points = extract_predicted_positive_points(msg, positive_color)
            metrics = evaluate_frame(predicted_points, gt_points, args.coordinate_tolerance)
            metrics["stamp"] = frame_stamp
            frame_results.append(metrics)
            matched_stamps.add(frame_stamp)

    missing_gt_frames = sorted(set(gt_files.keys()) - matched_stamps)
    if missing_gt_frames:
        print("[WARN] No prediction frame found for these GT stamps:")
        for frame_stamp in missing_gt_frames:
            print(f"  {frame_stamp}")

    return frame_results, missing_gt_frames, {"bag": str(bag_path), "topic": args.topic, "ground_truth_dir": str(gt_dir)}


def evaluate_pcd_pairs_mode(args, positive_color: tuple) -> tuple:
    pairs_list_path = Path(args.pairs_list).expanduser().resolve()
    if not pairs_list_path.exists():
        raise FileNotFoundError(f"Pairs list file not found: {pairs_list_path}")

    frame_results = []
    missing_gt_frames = []

    with pairs_list_path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 2:
                raise ValueError(f"Each line in pairs list must contain exactly two paths: {line}")
            pred_path = Path(parts[0]).expanduser().resolve()
            gt_path = Path(parts[1]).expanduser().resolve()

            if not pred_path.exists():
                raise FileNotFoundError(f"Prediction PCD not found: {pred_path}")
            if not gt_path.exists():
                raise FileNotFoundError(f"Ground-truth file not found: {gt_path}")

            gt_points = load_ground_truth_generic(gt_path)
            predicted_points = extract_predicted_positive_points_from_pcd(pred_path, positive_color)
            metrics = evaluate_frame(predicted_points, gt_points, args.coordinate_tolerance)
            frame_name = pred_path.stem
            metrics["stamp"] = frame_name
            frame_results.append(metrics)

    return frame_results, missing_gt_frames, {"pairs_list": str(pairs_list_path)}


def main() -> int:
    args = parse_args()
    if args.coordinate_tolerance <= 0:
        raise ValueError("--coordinate-tolerance must be > 0")

    positive_color = parse_positive_color(args.positive_color)

    if args.mode == "rosbag":
        if not args.bag or not args.ground_truth_dir:
            raise ValueError("--bag and --ground-truth-dir are required in rosbag mode")
        frame_results, missing_gt_frames, meta = evaluate_rosbag_mode(args, positive_color)
    elif args.mode == "pcd_pairs":
        if not args.pairs_list:
            raise ValueError("--pairs-list is required in pcd_pairs mode")
        frame_results, missing_gt_frames, meta = evaluate_pcd_pairs_mode(args, positive_color)
    else:
        raise ValueError(f"Unknown mode: {args.mode}")

    if not frame_results:
        print("[ERROR] No frames evaluated.", file=sys.stderr)
        return 1

    summary = summarize_frames(frame_results)
    print_summary(summary, frame_results)

    if args.output_json:
        output_path = Path(args.output_json).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "mode": args.mode,
            "coordinate_tolerance": args.coordinate_tolerance,
            "positive_color": list(positive_color),
            "summary": summary,
            "frames": frame_results,
            "missing_gt_frames": missing_gt_frames,
        }
        payload.update(meta)
        output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\n[INFO] Wrote JSON summary: {output_path}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # pragma: no cover - CLI error path
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)