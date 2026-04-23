#!/usr/bin/env python3
"""Add raw image topics to a new rosbag by decoding compressed image topics.

By default the script discovers all sensor_msgs/CompressedImage topics in the
bag, preserves every original message, and appends a decoded sensor_msgs/Image
message on a new topic with the same timestamp.
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

if sys.version_info[0] < 3:
    sys.stderr.write("decompress_bag_image_topics.py requires Python 3.\n")
    sys.exit(1)

try:
    import rosbag
    from sensor_msgs.msg import Image
except ImportError as exc:
    sys.stderr.write(
        "Failed to import ROS Python modules ({}). Source /opt/ros/noetic/setup.bash and devel/setup.bash first.\n".format(exc)
    )
    sys.exit(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rewrite a rosbag and add decoded sensor_msgs/Image topics for compressed camera topics.",
    )
    parser.add_argument("--input-bag", required=True, help="Source rosbag path")
    parser.add_argument(
        "--output-bag",
        default=None,
        help="Output rosbag path; defaults to <input>_with_raw.bag",
    )
    parser.add_argument(
        "--image-topic",
        action="append",
        default=None,
        help="Compressed image topic to decode. Repeat for multiple topics. Default: all sensor_msgs/CompressedImage topics in bag.",
    )
    parser.add_argument(
        "--output-topic",
        default=None,
        help="Output raw topic name. Only valid when exactly one --image-topic is provided. Default: infer from input topic.",
    )
    parser.add_argument(
        "--max-images-per-topic",
        type=int,
        default=None,
        help="Decode only the first N images for each selected topic. Original messages are still preserved.",
    )
    parser.add_argument(
        "--every-nth",
        type=int,
        default=1,
        help="Only convert every Nth compressed frame per topic (default: 1, convert all).",
    )
    parser.add_argument(
        "--target-fps",
        type=float,
        default=None,
        help="Limit converted Image output to target FPS per topic. Uses message timestamps.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Resize factor applied before writing Image (e.g., 0.5 halves width/height).",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=None,
        help="Maximum output image width in pixels (keeps aspect ratio).",
    )
    parser.add_argument(
        "--max-height",
        type=int,
        default=None,
        help="Maximum output image height in pixels (keeps aspect ratio).",
    )
    parser.add_argument(
        "--grayscale",
        action="store_true",
        help="Convert output Image to mono8 to reduce storage.",
    )
    parser.add_argument(
        "--drop-source-topic",
        action="store_true",
        help="Do not copy converted compressed source topic(s) into the output bag.",
    )
    parser.add_argument(
        "--bag-compression",
        choices=["none", "lz4", "bz2"],
        default="none",
        help="Compression for output bag chunks (default: none).",
    )
    parser.add_argument(
        "--progress-every",
        type=int,
        default=100,
        help="Print progress every N decoded images. Use 0 to disable.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Abort on the first decode or conversion failure instead of warning and continuing.",
    )
    return parser.parse_args()


def default_output_bag(input_bag: Path) -> Path:
    return input_bag.with_name(f"{input_bag.stem}_with_raw{input_bag.suffix}")


def infer_output_topic(input_topic: str) -> str:
    for suffix in ("/compressed", "/compressedDepth"):
        if input_topic.endswith(suffix):
            candidate = input_topic[: -len(suffix)]
            if candidate:
                return candidate
    return input_topic.rstrip("/") + "/raw"


def discover_compressed_topics(bag: rosbag.Bag) -> list:
    topic_info = bag.get_type_and_topic_info().topics
    return sorted(
        topic
        for topic, info in topic_info.items()
        if info.msg_type == "sensor_msgs/CompressedImage"
    )


def build_topic_map(args: argparse.Namespace, bag: rosbag.Bag) -> dict:
    topic_info = bag.get_type_and_topic_info().topics
    available_compressed = set(discover_compressed_topics(bag))

    if args.image_topic:
        selected_topics = []
        for topic in args.image_topic:
            if topic not in topic_info:
                raise ValueError(f"Input topic not found in bag: {topic}")
            if topic not in available_compressed:
                actual_type = topic_info[topic].msg_type
                raise ValueError(f"Input topic is not sensor_msgs/CompressedImage: {topic} ({actual_type})")
            selected_topics.append(topic)
    else:
        selected_topics = sorted(available_compressed)

    if not selected_topics:
        raise ValueError("No sensor_msgs/CompressedImage topics found in the bag")

    if args.output_topic and len(selected_topics) != 1:
        raise ValueError("--output-topic can only be used with exactly one --image-topic")

    topic_map = {}
    for topic in selected_topics:
        output_topic = args.output_topic if args.output_topic else infer_output_topic(topic)
        if output_topic == topic:
            raise ValueError(f"Output topic would overwrite the compressed topic: {topic}")
        topic_map[topic] = output_topic

    existing_topics = set(topic_info.keys())
    for input_topic, output_topic in topic_map.items():
        if output_topic in existing_topics:
            raise ValueError(
                "Output topic already exists in input bag: {} (from {}). Choose --output-topic explicitly.".format(
                    output_topic,
                    input_topic,
                )
            )

    return topic_map


def decode_compressed_image(msg) -> np.ndarray:
    format_hint = (msg.format or "").lower()
    if "compresseddepth" in format_hint:
        raise ValueError("compressedDepth transport is not supported by this script")

    data = np.frombuffer(msg.data, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("cv2.imdecode returned None; image data may be invalid")
    return image


def maybe_grayscale(image: np.ndarray, enable: bool) -> np.ndarray:
    if not enable:
        return image
    if image.ndim == 2:
        return image
    if image.ndim == 3:
        channels = image.shape[2]
        if channels == 1:
            return image[:, :, 0]
        if channels == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if channels == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    raise ValueError(f"Unsupported image shape for grayscale conversion: {image.shape}")


def maybe_resize(image: np.ndarray, scale: float, max_width: int, max_height: int) -> np.ndarray:
    height, width = image.shape[:2]

    resize_ratio = 1.0
    if scale != 1.0:
        resize_ratio *= scale

    if max_width is not None and width > 0:
        resize_ratio = min(resize_ratio, float(max_width) / float(width))
    if max_height is not None and height > 0:
        resize_ratio = min(resize_ratio, float(max_height) / float(height))

    if resize_ratio <= 0:
        raise ValueError(f"Invalid resize ratio: {resize_ratio}")

    if abs(resize_ratio - 1.0) < 1e-12:
        return image

    new_width = max(1, int(round(width * resize_ratio)))
    new_height = max(1, int(round(height * resize_ratio)))
    return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)


def transform_image(image: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    image = maybe_grayscale(image, args.grayscale)
    image = maybe_resize(image, args.scale, args.max_width, args.max_height)
    return image


def bag_compression_mode(name: str):
    if name == "none":
        return rosbag.Compression.NONE
    if name == "bz2":
        return rosbag.Compression.BZ2
    if name == "lz4":
        return rosbag.Compression.LZ4
    raise ValueError(f"Unsupported bag compression mode: {name}")


def ros_encoding_for_image(image: np.ndarray) -> str:
    if image.dtype == np.uint8:
        depth = "8"
    elif image.dtype == np.uint16:
        depth = "16"
    else:
        raise ValueError(f"Unsupported image dtype: {image.dtype}")

    if image.ndim == 2:
        return "mono" + depth

    if image.ndim != 3:
        raise ValueError(f"Unsupported image shape: {image.shape}")

    channels = image.shape[2]
    if channels == 1:
        return "mono" + depth
    if channels == 3:
        return "bgr" + depth
    if channels == 4:
        return "bgra" + depth

    raise ValueError(f"Unsupported channel count: {channels}")


def ros_is_bigendian(image: np.ndarray) -> int:
    if image.dtype.itemsize == 1:
        return 0
    if image.dtype.byteorder == ">":
        return 1
    if image.dtype.byteorder == "<":
        return 0
    return 1 if sys.byteorder == "big" else 0


def image_to_ros_msg(image: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = ros_encoding_for_image(image)
    msg.is_bigendian = ros_is_bigendian(image)
    msg.step = int(image.strides[0])
    msg.data = image.tobytes()
    return msg


def rewrite_bag(args: argparse.Namespace) -> int:
    input_bag = Path(args.input_bag).expanduser().resolve()
    output_bag = Path(args.output_bag).expanduser().resolve() if args.output_bag else default_output_bag(input_bag)

    if not input_bag.exists():
        raise FileNotFoundError(f"Input bag not found: {input_bag}")
    if input_bag == output_bag:
        raise ValueError("Output bag path must be different from input bag path")
    if args.max_images_per_topic is not None and args.max_images_per_topic < 0:
        raise ValueError("--max-images-per-topic must be >= 0")
    if args.every_nth < 1:
        raise ValueError("--every-nth must be >= 1")
    if args.target_fps is not None and args.target_fps <= 0:
        raise ValueError("--target-fps must be > 0")
    if args.scale <= 0:
        raise ValueError("--scale must be > 0")
    if args.max_width is not None and args.max_width <= 0:
        raise ValueError("--max-width must be > 0")
    if args.max_height is not None and args.max_height <= 0:
        raise ValueError("--max-height must be > 0")
    if args.progress_every < 0:
        raise ValueError("--progress-every must be >= 0")

    compression_mode = bag_compression_mode(args.bag_compression)

    output_bag.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(input_bag), "r") as in_bag:
        topic_map = build_topic_map(args, in_bag)
        source_counts = {topic: 0 for topic in topic_map}
        decoded_counts = {topic: 0 for topic in topic_map}
        failure_counts = {topic: 0 for topic in topic_map}
        time_skipped_counts = {topic: 0 for topic in topic_map}
        nth_skipped_counts = {topic: 0 for topic in topic_map}
        last_written_stamp = {topic: None for topic in topic_map}
        total_decoded = 0

        print(f"[INFO] input_bag:  {input_bag}")
        print(f"[INFO] output_bag: {output_bag}")
        print("[INFO] topic map:")
        for input_topic, output_topic in topic_map.items():
            print(f"  {input_topic} -> {output_topic}")

        with rosbag.Bag(str(output_bag), "w", compression=compression_mode) as out_bag:
            for topic, msg, timestamp in in_bag.read_messages():
                if not (args.drop_source_topic and topic in topic_map):
                    out_bag.write(topic, msg, timestamp)

                if topic not in topic_map:
                    continue

                source_counts[topic] += 1

                if args.max_images_per_topic is not None and decoded_counts[topic] >= args.max_images_per_topic:
                    continue

                if args.every_nth > 1 and (source_counts[topic] - 1) % args.every_nth != 0:
                    nth_skipped_counts[topic] += 1
                    continue

                if args.target_fps is not None:
                    prev = last_written_stamp[topic]
                    if prev is not None:
                        min_dt = 1.0 / args.target_fps
                        if (timestamp - prev).to_sec() < min_dt:
                            time_skipped_counts[topic] += 1
                            continue

                try:
                    image = decode_compressed_image(msg)
                    image = transform_image(image, args)
                    raw_msg = image_to_ros_msg(image, msg.header)
                except Exception as exc:
                    failure_counts[topic] += 1
                    if args.strict:
                        raise
                    if failure_counts[topic] <= 5:
                        print(f"[WARN] failed to decode {topic} at {timestamp.to_sec():.6f}: {exc}")
                    continue

                out_bag.write(topic_map[topic], raw_msg, timestamp)
                decoded_counts[topic] += 1
                last_written_stamp[topic] = timestamp
                total_decoded += 1

                if args.progress_every and total_decoded % args.progress_every == 0:
                    print(f"[INFO] decoded {total_decoded} images")

    print("[DONE] decoded image counts:")
    for topic in sorted(topic_map):
        print(
            "  {}: source={} decoded={} failed={} skipped_nth={} skipped_fps={}".format(
                topic,
                source_counts[topic],
                decoded_counts[topic],
                failure_counts[topic],
                nth_skipped_counts[topic],
                time_skipped_counts[topic],
            )
        )
    print(f"[DONE] wrote bag: {output_bag}")
    return 0


def main() -> int:
    args = parse_args()
    try:
        return rewrite_bag(args)
    except Exception as exc:
        sys.stderr.write(f"[ERROR] {exc}\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())