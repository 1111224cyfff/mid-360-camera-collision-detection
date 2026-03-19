import argparse
import pickle
from collections import OrderedDict

from ultralytics import YOLO


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-w',
                        '--weights',
                        type=str,
                        required=True,
                        help='YOLO11 pytorch weights')
    parser.add_argument('-o',
                        '--output',
                        type=str,
                        required=True,
                        help='Output file')
    args = parser.parse_args()
    return args


args = parse_args()

model = YOLO(args.weights)
model.model.fuse()
yolo_model = model.model.model

strides = yolo_model[-1].stride.detach().cpu().numpy()
reg_max = yolo_model[-1].dfl.conv.weight.shape[1]

state_dict = OrderedDict(GD=model.model.yaml['depth_multiple'],
                         GW=model.model.yaml['width_multiple'],
                         strides=strides,
                         reg_max=reg_max)

for name, value in yolo_model.state_dict().items():
    value = value.detach().cpu().numpy()
    i = int(name.split('.')[0])
    layer = yolo_model[i]
    module_name = layer.type.split('.')[-1]
    stem = module_name + '.' + name
    state_dict[stem] = value

with open(args.output, 'wb') as f:
    pickle.dump(state_dict, f)
