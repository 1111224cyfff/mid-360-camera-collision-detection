# yolo26_ros

这个包封装了仓库内置的 `src/yolo-inference/cpp` TensorRT 后端，并提供与现有 `yolo11_ros` 话题和启动方式兼容的 ROS 节点。

## 当前范围

- `detect` 节点：通过 `src/yolo-inference` 运行 YOLO26 TensorRT 检测
- `pose` 节点：通过 `src/yolo-inference` 运行 YOLO26 TensorRT 姿态估计
- `segment` 节点：通过 `src/yolo-inference` 运行 YOLO26 TensorRT 分割
- `system_bringup` 集成：支持通过 launch 参数覆盖分割流程

## 模型文件

TensorRT engine 文件默认放在以下位置：

- `src/yolo26_ros/weights/yolo26s.engine`
- `src/yolo26_ros/weights/yolo26s-pose.engine`
- `src/yolo26_ros/weights/yolo26s-seg.engine`

也可以通过现有的 `weight_name` 启动参数覆盖文件名。

## Engine 导出流程

后续复现你自己的模型时，需要先分清下面两种 engine：

- `raw engine`：用 `trtexec` 从 ONNX 直接生成的纯 TensorRT engine
- `Ultralytics-compatible engine`：在 raw engine 前面加上 Ultralytics metadata 头后的 engine，供 `yolo predict model=...engine` 正确解析

### 为什么要区分

- 可能会出现 `pt` 和 `onnx` 预测一致，但 `trtexec` 生成的 `engine` 在 Ultralytics CLI 或 Python 里结果差很多
- 这种情况通常不是模型精度真的退化了
- 常见根因是 engine 缺少 Ultralytics metadata，导致任务类型、类别信息、后处理方式被错误解码，最后出现离谱的类别 ID 或置信度

当前仓库已经同时兼容这两种格式：

- ROS TensorRT 加载器可以直接读取 raw engine
- ROS TensorRT 加载器也可以读取带 Ultralytics metadata 的 engine
- Ultralytics CLI 和 `ultralytics.YOLO(...engine)` 则要求使用 Ultralytics-compatible engine

### 推荐复现路径

1. 先训练或准备好 `.pt` 模型。
2. 从 `.pt` 导出 `.onnx`。
3. 用 `trtexec` 从 `.onnx` 构建 raw TensorRT engine。
4. 如果还要在 Ultralytics CLI 或 Python 中直接加载 `.engine`，再把 raw engine 包装成带 metadata 的版本。
5. 在同一张图片上对比 `pt`、`onnx`、包装后的 `engine`，确认三者结果一致后再接入 ROS。

### 示例命令

从 PT 导出 ONNX：

```bash
cd /home/nvidia/ws_livox/src/yolo26_ros/weights
yolo export model=yolo26s-seg.pt format=onnx imgsz=640 simplify=True opset=12 dynamic=False
```

从 ONNX 构建 raw TensorRT engine：

```bash
cd /home/nvidia/ws_livox
/usr/src/tensorrt/bin/trtexec \
  --onnx=src/yolo26_ros/weights/yolo26s-seg.onnx \
  --saveEngine=src/yolo26_ros/weights/yolo26s-seg.engine \
  --fp16 \
  --workspace=1024 \
  --buildOnly
```

给 raw engine 添加 Ultralytics metadata：

```bash
cd /home/nvidia/ws_livox
/home/nvidia/ws_livox/.venv/bin/python scripts/wrap_engine_with_ultralytics_metadata.py \
  --pt src/yolo26_ros/weights/yolo26s-seg.pt \
  --engine src/yolo26_ros/weights/yolo26s-seg.engine \
  --half
```

执行后会生成：

- `src/yolo26_ros/weights/yolo26s-seg.engine`：raw engine
- `src/yolo26_ros/weights/yolo26s-seg.ultra.engine`：Ultralytics-compatible engine

### 该用哪个文件

下面这些场景用 raw engine：

- 运行 `yolo26_ros` 的 ROS 节点
- 直接运行仓库里的 `src/yolo-inference` TensorRT 后端

下面这些场景用 `.ultra.engine`：

- 运行 `yolo predict model=...engine`
- 使用 `from ultralytics import YOLO` 直接加载 `.engine`

### 快速验证

在同一张图片上跑三种格式：

```bash
cd /home/nvidia/ws_livox/src/yolo26_ros/weights
yolo predict model=yolo26s-seg.pt source=/path/to/test.jpg save=True
yolo predict model=yolo26s-seg.onnx source=/path/to/test.jpg save=True
yolo predict model=yolo26s-seg.ultra.engine source=/path/to/test.jpg save=True
```

预期结果：

- 目标数量应接近或一致
- 类别 ID 应保持一致
- 框的位置和分割 mask 应基本对齐

如果 `pt` 和 `onnx` 一致，但 raw `.engine` 只在 Ultralytics 里表现得很差，优先按 metadata 格式问题处理，不要先假设这是 TensorRT 精度退化。

## 编译

在工作区根目录执行：

```bash
catkin_make --pkg yolo26_ros -j2
```

## 启动

单独启动：

```bash
roslaunch yolo26_ros detect.launch
roslaunch yolo26_ros pose.launch
roslaunch yolo26_ros segment.launch
```

在不改全局默认值的情况下，把系统分割流程切换到 YOLO26：

```bash
roslaunch system_bringup run.launch \
  segment_launch_file:=$(rospack find yolo26_ros)/launch/segment.launch \
  segment_projection_config_path:=$(rospack find yolo26_ros)/config/segment_projection.yaml \
  segment_weight_name:=yolo26s-seg.engine
```

下面这些 launch 也支持同样的 `segment_launch_file` 覆盖方式：

- `system_bringup/launch/run.launch`
- `system_bringup/launch/run_segment.launch`
- `system_bringup/launch/run_live.launch`

## 说明

- 当前包默认沿用工作区里 `yolo11_ros` 现有的 TensorRT/CUDA/OpenCV 环境：TensorRT `8.5.1.7`、CUDA `11.4`、OpenCV `4.5`。
- `src/yolo-inference` 已做本地补丁，暴露了 ROS 封装需要的内存内 `cv::Mat` 推理接口和结果读取接口。
- `src/yolo-inference/cpp/tensorrt/yolo_tensorrt.cpp` 和 `src/yolo-inference/python/backends/TensorRT/yolo_tensorrt.py` 已补丁为同时支持 raw engine 和带 Ultralytics metadata 的 engine。
- 给 raw engine 加 metadata 的辅助脚本是 `scripts/wrap_engine_with_ultralytics_metadata.py`。