'''
Author: taifyang  
Date: 2024-06-12 22:23:07
LastEditTime: 2025-12-23 08:37:29
Description: tensorrt inference class for YOLO algorithm
'''

import tensorrt as trt
from backends.yolo import *
from backends.utils import *


def strip_ultralytics_engine_metadata(engine_bytes: bytes) -> bytes:
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


'''
description: tensorrt inference class for YOLO algorithm
'''
class YOLO_TensorRT(YOLO):
    '''
    description:            construction method
    param {*} self          instance of class
    param {str} algo_type   algorithm type
    param {str} device_type device type
    param {str} model_type  model type
    param {str} model_path  model path
    return {*}
    '''     
    def __init__(self, algo_type:str, device_type:str, model_type:str, model_path:str) -> None:
        super().__init__()
        assert os.path.exists(model_path), 'model not exists!'
        assert device_type in ['GPU'], 'unsupported device type!'
        self.algo_type = algo_type
        self.model_type = model_type
        logger = trt.Logger(trt.Logger.ERROR)
        with open(model_path, 'rb') as f, trt.Runtime(logger) as runtime:
            assert runtime, 'runtime create failed!'
            self.engine = runtime.deserialize_cuda_engine(strip_ultralytics_engine_metadata(f.read()))
        assert self.engine, 'engine create failed!'
        self.context = self.engine.create_execution_context()
        assert self.context, 'context create failed!'

        self.input_shapes = [] 
        self.outputs_shape = [] 
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                shape = self.engine.get_tensor_shape(name)
                self.input_shapes.append(shape)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
                shape = self.engine.get_tensor_shape(name)
                self.outputs_shape.append(shape)
