import argparse

from ultralytics import YOLO


def parse_args():
	parser = argparse.ArgumentParser()
	parser.add_argument('-w',
						'--weights',
						type=str,
						required=True,
						help='PyTorch yolo11 pose weights')
	return parser.parse_args()


def main():
	args = parse_args()
	model = YOLO(args.weights)
	success = model.export(format="onnx", opset=11, simplify=True)
	assert success


if __name__ == '__main__':
	main()