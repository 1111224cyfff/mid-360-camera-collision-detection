#!/usr/bin/env python3

import argparse
import os
import sys

if sys.version_info[0] < 3:
    sys.stderr.write(
        "adjust_bag_exposure.py requires Python 3. Use 'python3' or '/home/nvidia/ws_livox/.venv/bin/python' after sourcing ROS Noetic.\n"
    )
    sys.exit(1)

import cv2
import numpy as np

try:
    import rosbag
    from sensor_msgs.msg import CompressedImage
except ImportError as exc:
    sys.stderr.write(
        "Failed to import ROS Python modules ({}). Source /opt/ros/noetic/setup.bash and devel/setup.bash first.\n".format(exc)
    )
    sys.exit(1)


def decode_compressed_image(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("cv2.imdecode returned None; message data may be invalid")
    return image


def encode_compressed_image(image, original_msg, jpeg_quality, png_compression):
    format_hint = (original_msg.format or "jpeg").lower()
    is_png = "png" in format_hint
    extension = ".png" if is_png else ".jpg"
    encode_params = [cv2.IMWRITE_PNG_COMPRESSION, png_compression] if is_png else [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
    ok, encoded = cv2.imencode(extension, image, encode_params)
    if not ok:
        raise IOError("cv2.imencode failed while writing adjusted image")

    out = CompressedImage()
    out.header = original_msg.header
    out.format = original_msg.format or ("png" if is_png else "jpeg")
    out.data = np.asarray(encoded).tobytes()
    return out


def apply_exposure_adjustment(image, alpha, gamma, clahe_clip_limit, clahe_tile_size):
    adjusted = np.clip(image.astype(np.float32) * alpha, 0.0, 255.0) / 255.0
    if gamma != 1.0:
        adjusted = np.power(adjusted, gamma)
    adjusted = np.clip(adjusted * 255.0, 0.0, 255.0).astype(np.uint8)

    if clahe_clip_limit <= 0.0:
        return adjusted

    clahe = cv2.createCLAHE(clipLimit=clahe_clip_limit, tileGridSize=(clahe_tile_size, clahe_tile_size))
    if adjusted.ndim == 2:
        return clahe.apply(adjusted)

    if adjusted.ndim == 3 and adjusted.shape[2] == 1:
        single = clahe.apply(adjusted[:, :, 0])
        return single[:, :, np.newaxis]

    lab = cv2.cvtColor(adjusted, cv2.COLOR_BGR2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    l_channel = clahe.apply(l_channel)
    return cv2.cvtColor(cv2.merge((l_channel, a_channel, b_channel)), cv2.COLOR_LAB2BGR)


def default_output_bag(input_bag):
    input_dir = os.path.dirname(input_bag)
    input_name = os.path.basename(input_bag)
    stem, suffix = os.path.splitext(input_name)
    return os.path.join(input_dir, "{}_adjusted{}".format(stem, suffix))


def adjust_bag(args):
    output_bag = args.output_bag or default_output_bag(args.input_bag)
    output_topic = args.output_topic or args.image_topic

    processed_images = 0
    total_messages = 0
    with rosbag.Bag(args.input_bag, "r") as in_bag, rosbag.Bag(output_bag, "w") as out_bag:
        for topic, msg, timestamp in in_bag.read_messages():
            total_messages += 1
            if topic != args.image_topic:
                out_bag.write(topic, msg, timestamp)
                continue

            if args.max_images is not None and processed_images >= args.max_images:
                out_bag.write(output_topic, msg, timestamp)
                continue

            image = decode_compressed_image(msg)
            adjusted = apply_exposure_adjustment(
                image,
                alpha=args.alpha,
                gamma=args.gamma,
                clahe_clip_limit=args.clahe_clip_limit,
                clahe_tile_size=args.clahe_tile_size,
            )
            adjusted_msg = encode_compressed_image(
                adjusted,
                original_msg=msg,
                jpeg_quality=args.jpeg_quality,
                png_compression=args.png_compression,
            )
            out_bag.write(output_topic, adjusted_msg, timestamp)
            processed_images += 1

            if processed_images % args.progress_every == 0:
                print("processed {} images / {} total messages".format(processed_images, total_messages))

    print("wrote {} adjusted images to {}".format(processed_images, output_bag))
    print("image topic: {} -> {}".format(args.image_topic, output_topic))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Rewrite a rosbag with darkened compressed camera images while preserving all other topics and timestamps."
    )
    parser.add_argument("--input-bag", required=True, help="Source rosbag path")
    parser.add_argument("--output-bag", default=None, help="Output rosbag path; defaults to <input>_adjusted.bag")
    parser.add_argument("--image-topic", default="/hikrobot_camera/rgb/compressed", help="Compressed image topic to rewrite")
    parser.add_argument("--output-topic", default=None, help="Optional replacement topic in the output bag; defaults to the input topic")
    parser.add_argument("--alpha", type=float, default=0.5, help="Linear darkening scale; smaller than 1.0 makes the image darker")
    parser.add_argument("--gamma", type=float, default=1.4, help="Gamma curve; larger than 1.0 darkens mid-tones")
    parser.add_argument("--clahe-clip-limit", type=float, default=1.0, help="CLAHE clip limit; 0 disables local contrast enhancement")
    parser.add_argument("--clahe-tile-size", type=int, default=8, help="CLAHE tile size in pixels")
    parser.add_argument("--jpeg-quality", type=int, default=95, help="JPEG quality for rewritten images")
    parser.add_argument("--png-compression", type=int, default=3, help="PNG compression level if the source topic uses PNG")
    parser.add_argument("--max-images", type=int, default=None, help="Rewrite only the first N images on the image topic")
    parser.add_argument("--progress-every", type=int, default=100, help="Print progress every N processed images")
    return parser.parse_args()


if __name__ == "__main__":
    adjust_bag(parse_args())