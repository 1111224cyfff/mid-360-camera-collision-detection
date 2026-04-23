# YOLO26 自训 Engine 接入修正记录（2026-04-15）

## 背景

在 `yolo26_ros` 中替换官方分割权重为自训权重后，出现了以下现象：

- `yolo segment predict model=best.ultra.engine ...` 在 Ultralytics CLI 中可以正常输出结果。
- `roslaunch system_bringup run.launch` 使用同一套自训 TensorRT engine 时，`segment` 节点没有正常预测结果输出。
- 切回官方 `yolo26s-seg.engine` 时，ROS 流程恢复正常。

这说明问题不在模型训练本身，而在 `yolo26_ros` 的 TensorRT 接入或 ROS 侧标签映射逻辑。

## 本次修改概览

本次改动涉及三个方面：

1. 修复 `segment` 节点硬编码类别名问题。
2. 为 launch 增加可配置 `class_names` 参数。
3. 修复自训 TensorRT segmentation engine 与官方 engine 输出 binding 顺序不一致导致的无输出问题。

## 1. 去除分割节点中的硬编码类别名

修改文件：

- `src/yolo26_ros/src_ros/segment.cpp`

原先 `segment.cpp` 中存在固定的 `CLASS_NAMES` 表，内容混合了旧项目类别和 COCO 类别，其中包含 `umbrella`。这会带来两个问题：

- 当模型类别数与硬编码表不一致时，显示和发布的 `class_name` 会错位。
- 即使模型本身只有 2 类，也可能在 ROS 输出中看到并不存在于标注集中的标签，如 `umbrella`。

修正后：

- 删除固定 `CLASS_NAMES`。
- 改为从 ROS 参数 `class_names` 读取类别名。
- 当 `class_names` 未提供或索引越界时，回退为 `class0`、`class1` 这类安全标签。
- 绘制框和发布 `SegmentedObjectStateArray` 时统一使用新的安全解析逻辑。
- 同时对颜色索引做了边界保护，避免类别索引越界造成崩溃或未定义行为。

关键函数：

- `resolveClassName(...)`
- `readClassNamesParam(...)`

## 2. 在 launch 中增加 `class_names` 参数

修改文件：

- `src/yolo26_ros/launch/segment.launch`

新增内容：

- `class_names` launch 参数。
- 通过 `<rosparam param="class_names" ...>` 传给节点。

当前默认值已设置为：

```xml
<arg name="class_names" default="['tower_crane_section','barrier']" />
```

这样即使直接运行 `roslaunch yolo26_ros segment.launch`，也会优先使用当前自训模型的类别名。

## 3. 修复自训 engine 与官方 engine 的输出 binding 顺序差异

修改文件：

- `src/yolo-inference/cpp/tensorrt/yolo_tensorrt_segment.cpp`

### 现象

对官方 engine 和自训 engine 进行 TensorRT I/O 检查后，发现二者虽然输出张量形状一致，但输出顺序不同：

官方 engine：

- input: `(1, 3, 640, 640)`
- `output1`: `(1, 32, 160, 160)`
- `output0`: `(1, 300, 38)`

自训 engine：

- input: `(1, 3, 640, 640)`
- `output0`: `(1, 300, 38)`
- `output1`: `(1, 32, 160, 160)`

原实现按固定索引绑定输出指针，默认假设 segmentation proto 和 detection output 的顺序是固定的。这在官方 engine 上恰好成立，但在自训 engine 上会把两个输出绑反，导致：

- 后处理读取错张量
- 检测框与 mask 解码失效
- ROS 节点表现为“启动正常但无预测结果”

### 修正方式

新增 `dimsNumel(...)`，按 tensor shape 自动识别输出类型：

- `300 x 38` 对应 detection output
- `32 x 160 x 160` 对应 segmentation proto output

在 TensorRT 8/10 两套 API 分支中，均改为：

- 遍历 binding / io tensor
- 输入 tensor 绑定到 `m_input_device`
- 输出 tensor 根据 `numel` 自动绑定到 `m_output0_device` 或 `m_output1_device`

这样就不再依赖 engine 序列化时的输出顺序，官方 engine 和自训 engine 都能兼容。

## 验证结果

### Ultralytics 侧

以下链路已验证通过：

- `best.pt`
- `best.onnx`
- `best.engine`
- `best.ultra.engine`

其中：

- `best.ultra.engine` 可在 Ultralytics CLI 中正常预测。
- 类别名已确认是 `tower_crane_section` 和 `barrier`，不再出现 `umbrella` 之类错误标签。

### ROS 编译

执行：

```bash
catkin_make --pkg yolo26_ros -j2
```

已编译通过。

### ROS 全流程

在 `run.launch` 的短时 smoke test 中，能够确认：

- `class_names` 已正确注入参数服务器。
- `segment` 相关节点成功启动。

测试日志中仍可见 `body_leveled` / `map` 等 TF 与 leveler 初始化等待信息，这属于系统整体启动链路中的上游时序问题，不是这次自训 engine 无预测输出的根因。

## 当前结论

本次“官方权重正常、自训权重无输出”的直接根因是：

- 自训 TensorRT segmentation engine 的输出 binding 顺序与官方 engine 不同；
- 原接入代码按固定顺序绑定输出，导致后处理读错张量；
- 同时 `segment.cpp` 的硬编码类名表放大了问题表现，造成错误类别名显示。

修正后：

- `segment` 节点不再依赖硬编码类名表；
- launch 可显式配置类别名；
- TensorRT segmentation 后处理不再依赖固定输出顺序；
- 官方和自训 engine 均可共用同一套 `yolo26_ros` 接入逻辑。

## 后续建议

1. 若后续切换新的自训模型，优先检查 engine I/O 形状和输出顺序，不要默认与官方模型完全一致。
2. 若继续使用 `system_bringup run.launch` 做整链路验证，建议单独观察以下话题是否稳定：

- `/hikrobot_camera/rgb/leveled`
- `/merged_pointcloud_leveled`
- `/segment/image_raw`
- `/segment/object_states`

3. 若需要进一步降低接入风险，可在 `segment` 节点启动时额外打印：

- 当前 engine 文件路径
- 读到的类别名列表长度
- TensorRT 输出 tensor 形状

这样现场更容易区分“模型问题”和“系统时序问题”。