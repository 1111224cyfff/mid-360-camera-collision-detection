#!/usr/bin/env python3

import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import List

from ultralytics import YOLO, __version__


def build_metadata(model: YOLO, imgsz: List[int], batch: int, dynamic: bool, half: bool) -> dict:
    core = model.model
    stride = core.stride
    if hasattr(stride, 'tolist'):
        stride = stride.tolist()

    return {
        'description': f'Ultralytics {Path(core.yaml.get("yaml_file", model.ckpt_path or "model")).stem.replace("yolo", "YOLO")} model',
        'author': 'Ultralytics',
        'date': datetime.now().isoformat(),
        'version': __version__,
        'license': 'AGPL-3.0 License (https://ultralytics.com/license)',
        'docs': 'https://docs.ultralytics.com',
        'stride': int(max(stride)),
        'task': model.task,
        'batch': batch,
        'imgsz': imgsz,
        'names': model.names,
        'args': {
            'batch': batch,
            'dynamic': dynamic,
            'half': half,
            'nms': False,
        },
        'channels': int(core.yaml.get('channels', 3)),
        'end2end': bool(getattr(core, 'end2end', False)),
    }


def strip_existing_metadata(engine_bytes: bytes) -> bytes:
    if len(engine_bytes) <= 4:
        return engine_bytes

    meta_len = int.from_bytes(engine_bytes[:4], byteorder='little', signed=True)
    if meta_len <= 0 or meta_len > 1024 * 1024:
        return engine_bytes

    prefix_end = 4 + meta_len
    if prefix_end >= len(engine_bytes):
        return engine_bytes

    try:
        metadata = engine_bytes[4:prefix_end].decode('utf-8')
    except UnicodeDecodeError:
        return engine_bytes

    if '"task"' not in metadata or '"imgsz"' not in metadata or '"names"' not in metadata:
        return engine_bytes

    return engine_bytes[prefix_end:]


def main() -> int:
    parser = argparse.ArgumentParser(description='Wrap a raw TensorRT engine with Ultralytics metadata.')
    parser.add_argument('--pt', required=True, help='Source Ultralytics .pt model used to derive metadata.')
    parser.add_argument('--engine', required=True, help='Input raw TensorRT engine file.')
    parser.add_argument('--output', help='Output engine path. Defaults to <engine_stem>.ultra.engine')
    parser.add_argument('--imgsz', nargs=2, type=int, default=[640, 640], metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--batch', type=int, default=1)
    parser.add_argument('--dynamic', action='store_true')
    parser.add_argument('--half', action='store_true')
    args = parser.parse_args()

    engine_path = Path(args.engine)
    output_path = Path(args.output) if args.output else engine_path.with_name(f'{engine_path.stem}.ultra{engine_path.suffix}')

    model = YOLO(args.pt)
    metadata = build_metadata(model, args.imgsz, args.batch, args.dynamic, args.half)

    raw_engine = strip_existing_metadata(engine_path.read_bytes())
    meta_bytes = json.dumps(metadata).encode('utf-8')
    output_path.write_bytes(len(meta_bytes).to_bytes(4, byteorder='little', signed=True) + meta_bytes + raw_engine)

    print(f'Wrote Ultralytics-compatible engine to {output_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())