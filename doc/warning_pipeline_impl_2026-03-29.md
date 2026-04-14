# 多层级预警链路第一阶段实现记录（2026-03-29）

## 1. 本次实现目标

- 将现有工程从“分割/投影结果可视化 + 动态轨迹可视化”推进到“可机读的预警输入 + 独立预警评估节点”。
- 预警逻辑不再复用旧版 `yolo11_ros` 的二维网格报警思路，而是转为：
  - 分割侧发布吊装构件三维对象状态。
  - 动态跟踪侧发布带速度与轨迹 ID 的结构化状态。
  - 预警侧独立融合静态距离、动态趋势和 TTC，输出三级预警状态。
- 系统默认分割路径统一收敛到 `yolo26_ros`，移除 `system_bringup` 中对 `yolo11_ros` 的运行依赖。

## 2. 已完成的代码改动

### 2.1 `lio_sam` 中新增消息定义

新增消息：

- `DynamicTrack.msg`
- `DynamicTrackArray.msg`
- `SegmentedObjectState.msg`
- `SegmentedObjectStateArray.msg`
- `WarningState.msg`

目的：

- 给动态跟踪输出统一的 `track_id / state / velocity / speed / misses` 字段。
- 给分割投影输出统一的 `class / confidence / pose / size / bounding_radius / nearest_range / point_count` 字段。
- 给预警模块输出统一的 `active_level / desired_level / source / reason / clearance / ttc / head_on` 字段。

### 2.2 `dynamicTracker.cpp` 发布结构化动态轨迹

文件：`src/LIO-SAM-MID360/src/dynamicTracker.cpp`

新增：

- 发布话题 `/dynamic_tracker/track_states`
- 消息类型 `lio_sam/DynamicTrackArray`

当前输出内容：

- 目标 ID
- Tentative / Confirmed 状态
- 当前位置
- 速度向量与标量速度
- 轨迹年龄
- 距离最近一次更新的时间
- 丢失帧计数

这一步解决了原先 `PoseArray` 只能可视化、不能严肃参与预警判级的问题。

### 2.3 `yolo26_ros` 分割节点发布结构化对象状态

文件：`src/yolo26_ros/src_ros/segment.cpp`

新增：

- 发布话题 `/segment/object_states`
- 消息类型 `lio_sam/SegmentedObjectStateArray`

对象状态由分割掩码关联得到的实例点云集合构建，当前包含：

- `class_id / class_name`
- `confidence`
- 3D 质心 `pose`
- 包围尺寸 `size`
- 空间包络半径 `bounding_radius`
- 到传感器原点的最近距离 `nearest_range`
- 关联点数量 `point_count`

这一步把原来仅存在于分割节点内部的三维定位结果，正式转换成了可供下游预警消费的结构化接口。

### 2.4 新增独立预警节点 `warningEvaluator`

文件：`src/LIO-SAM-MID360/src/warningEvaluator.cpp`

输入：

- `/segment/object_states`
- `/dynamic_tracker/track_states`
- `/lio_sam/mapping/map_local`

输出：

- `/warning/state`
- `/warning/markers`

当前实现逻辑：

1. 从分割对象中选出监控目标。
2. 将目标位姿统一到 `output_frame`。
3. 基于相邻帧位置差估计吊物速度。
4. 用局部地图最近邻距离计算静态净空。
5. 对每个动态目标计算：
   - 当前净空
   - 相对接近速度
   - TTC
   - 预测最近接净空
   - 是否属于相向运动升级场景
6. 静态等级和动态等级取最大值。
7. 使用迟滞机制控制等级下降，避免抖动。

当前预警级别：

- `LEVEL_NOTICE`
- `LEVEL_WARNING`
- `LEVEL_EMERGENCY`

当前输出说明：

- `desired_level` 表示规则计算得到的即时等级。
- `active_level` 表示经过迟滞后的实际输出等级。

### 2.5 `system_bringup` 默认切换到 `yolo26_ros`

调整文件：

- `src/system_bringup/package.xml`
- `src/system_bringup/launch/run.launch`
- `src/system_bringup/launch/run_live.launch`

本次修改：

- 移除 `system_bringup` 对 `yolo11_ros` 的 `exec_depend`
- 默认分割权重改为 `yolo26s-seg.engine`
- 默认分割 launch 改为 `$(find yolo26_ros)/launch/segment.launch`
- 默认投影配置改为 `$(find yolo26_ros)/config/segment_projection.yaml`
- `run.launch` 的同步窗口默认值调整为 `0.05` 秒，与当前 YOLO26 路径保持一致
- 删除已无效的 `run_segment_yolo11.launch`
- 在 `src/yolo11_ros` 下新增 `CATKIN_IGNORE`，避免全量 `catkin_make` 时再次把历史包编进工作区

## 3. 当前系统结构

### 3.1 感知与预警主链路

```text
YOLO26 分割节点
  -> /segment/object_states

M-detector + dynamicTracker
  -> /dynamic_tracker/track_states

LIO-SAM 局部地图
  -> /lio_sam/mapping/map_local

warningEvaluator
  -> /warning/state
  -> /warning/markers
```

### 3.2 当前判级原则

- 静态风险：基于目标与局部地图最近邻点的净空距离。
- 动态风险：基于当前净空、预测净空、相对接近速度和 TTC。
- 相向运动：在已有动态风险等级基础上升级一级，但不超过 `EMERGENCY`。
- 迟滞降级：避免等级在阈值附近频繁跳变。

## 4. 当前阶段的局限

- 当前分割对象速度仍由相邻帧差分估计，尚未融合吊钩运动学或 IMU 约束。
- 静态净空使用 `map_local` 最近邻距离，仍偏工程近似，尚未构建更稳定的障碍物空间包络表达。
- 动态目标目前按球形半径近似，尚未使用聚类尺寸或目标朝向。
- 预警节点当前只输出状态和 RViz marker，尚未下发控制接口或停机指令。
- `warning_evaluator.yaml` 中的阈值属于第一阶段默认值，需要结合 rosbag 复盘进一步整定。

## 5. 建议的后续迭代顺序

### 阶段 2

- 将动态目标尺寸从固定半径扩展为聚类包络尺寸。
- 给目标状态增加更稳定的速度/航向估计。
- 将对象筛选从“最近且可信”扩展为“主吊物优先 + 多目标并行评估”。

### 阶段 3

- 定义控制接口消息，例如限速、限向、急停标志位。
- 形成 `warning -> control_command` 的闭环验证链路。
- 增加 rosbag 离线评估脚本，统计触发距离、TTC、误报和漏报。

## 6. 当前构建建议

由于 `yolo11_ros` 已经移除，构建时不要再包含该包名。当前建议命令：

```bash
cd /home/nvidia/ws_livox
catkin_make --pkg lio_sam yolo26_ros system_bringup
```

## 7. 当前运行建议

### bag 复现整链路

```bash
roslaunch system_bringup run.launch
```

### 在线运行整链路

```bash
roslaunch system_bringup run_live.launch enable_segment:=true enable_warning_evaluator:=true
```

## 8. 结论

本次实现已经把项目从“有感知结果”推进到“有统一预警输入和初版多层级预警输出”。

就工程分层而言，目前最重要的结构已经建立：

- 分割节点负责对象感知与三维定位输出。
- 动态跟踪节点负责目标级运动状态输出。
- 预警节点负责规则融合和状态机管理。

这个分层是后续继续加 TTC 细化、控制接口和实验统计时最稳妥的基础。