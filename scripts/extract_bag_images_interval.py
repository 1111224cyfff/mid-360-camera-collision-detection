#!/usr/bin/env python3
"""Extract images from a ROS bag at a fixed time interval.

Default behavior matches the recent task:
- topic: /hikrobot_camera/rgb/compressed
- interval: 1.0 second
- output: same directory as bag file
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract one image every N seconds from a bag image topic.",
    )
    parser.add_argument("bag", help="Path to .bag file")
    parser.add_argument(
        "--topic",
        default="/hikrobot_camera/rgb/compressed",
        help="Image topic in bag (default: /hikrobot_camera/rgb/compressed)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sampling interval in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory (default: same folder as bag)",
    )
    parser.add_argument(
        "--prefix",
        default=None,
        help="Output filename prefix (default: bag basename)",
    )
    parser.add_argument(
        "--ext",
        default="jpg",
        choices=["jpg", "png"],
        help="Output image extension (default: jpg)",
    )
    return parser.parse_args()


def decode_image(msg) -> np.ndarray:
    if hasattr(msg, "format") and hasattr(msg, "data"):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    if hasattr(msg, "encoding") and hasattr(msg, "data"):
        channels = 3 if "rgb" in msg.encoding.lower() or "bgr" in msg.encoding.lower() else 1
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, channels))
        if "rgb" in msg.encoding.lower() and channels == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    return None


def main() -> int:
    args = parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        print(f"[ERROR] Bag file not found: {bag_path}")
        return 1

    if args.interval <= 0:
        print("[ERROR] --interval must be > 0")
        return 1

    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else bag_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    prefix = args.prefix if args.prefix else bag_path.stem

    saved = 0
    decode_failed = 0
    last_saved_t = None

    print(f"[INFO] bag     : {bag_path}")
    print(f"[INFO] topic   : {args.topic}")
    print(f"[INFO] interval: {args.interval}s")
    print(f"[INFO] out_dir : {out_dir}")

    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, t in bag.read_messages(topics=[args.topic]):
            ts = t.to_sec()
            if last_saved_t is not None and (ts - last_saved_t) < args.interval:
                continue

            img = decode_image(msg)
            if img is None:
                decode_failed += 1
                continue

            out_name = f"{prefix}_{args.interval:g}s_{saved:04d}_{ts:.3f}.{args.ext}"
            out_path = out_dir / out_name
            if not cv2.imwrite(str(out_path), img):
                print(f"[WARN] Failed to write: {out_path}")
                continue

            saved += 1
            last_saved_t = ts

    print(f"[DONE] Saved {saved} images")
    if decode_failed:
        print(f"[WARN] Decode failed: {decode_failed}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
