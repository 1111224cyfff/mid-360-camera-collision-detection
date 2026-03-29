# 预警到控制接口原型实现记录（2026-03-29）

## 1. 本次实现目标

- 在现有 `/warning/state` 的基础上，补一层可机读的控制接口输出。
- 先实现“预警状态到控制命令”的原型桥接，不直接接入实机控制器。
- 控制层输出保持保守：只发布 `control_command` 话题，不主动调用任何硬件接口。

## 2. 设计思路

本次不把控制策略塞回 `warningEvaluator`，而是单独新增 `warning_controller` 节点，原因如下：

- `warningEvaluator` 的职责应当保持在风险判定和状态机层。
- 控制接口的字段和动作映射未来还会反复调整，独立节点更容易替换。
- 后续如果要接 PLC、上位机或底盘控制器，只需要替换 `warning_controller` 的下游，而不必改动风险评估本身。

当前链路变为：

```text
/warning/state
  -> warning_controller
  -> /warning/control_command
```

## 3. 新增消息

文件：`src/LIO-SAM-MID360/msg/ControlCommand.msg`

### 3.1 动作枚举

- `ACTION_CLEAR`
- `ACTION_OBSERVE`
- `ACTION_LIMIT_MOTION`
- `ACTION_HOLD_POSITION`
- `ACTION_EMERGENCY_STOP`

### 3.2 主要字段

- `warning_level`: 对应上游预警等级
- `action`: 控制动作枚举
- `action_text`: 便于日志和调试的动作文本
- `speed_limit_ratio`: 限速比例，范围 `[0, 1]`
- `hold_position`: 是否要求保持当前位置
- `emergency_stop`: 是否触发紧急停止
- `operator_ack_required`: 是否要求人工确认
- `recommended_direction`: 建议的退让方向
- `monitored_pose` / `monitored_velocity`: 透传被监控目标状态

## 4. `warning_controller` 节点逻辑

文件：`src/LIO-SAM-MID360/src/warningController.cpp`

### 4.1 输入输出

输入：

- `/warning/state`

输出：

- `/warning/control_command`

### 4.2 当前动作映射

#### 清除状态

- 条件：`warning_level == LEVEL_NONE`
- 输出：`ACTION_CLEAR`
- 行为：恢复正常、限速比例为 `1.0`

#### 一级提示

- 条件：`warning_level == LEVEL_NOTICE`
- 输出：`ACTION_OBSERVE`
- 行为：只提示，不下发运动限制

#### 二级报警

- 条件：`warning_level == LEVEL_WARNING`
- 默认输出：`ACTION_LIMIT_MOTION`
- 默认行为：下发限速比例 `warning_speed_limit_ratio`
- 可选策略：若参数配置开启，可在静态或融合风险下改成 `ACTION_HOLD_POSITION`

#### 三级紧急

- 条件：`warning_level == LEVEL_EMERGENCY`
- 输出：`ACTION_EMERGENCY_STOP`
- 行为：
  - `speed_limit_ratio = 0`
  - `hold_position = true`
  - `emergency_stop = true`

### 4.3 建议退让方向

当前原型策略比较保守：

- 当预警等级达到 `WARNING` 及以上时，若被监控目标自身速度足够大，则输出与当前速度相反的单位方向向量。
- 这等价于“建议先逆当前运动方向减速/退让”。

该实现是原型策略，不代表最终实机避障方向规划。

## 5. 新增配置与启动文件

### 5.1 新增配置

文件：`src/LIO-SAM-MID360/config/warning_controller.yaml`

当前可调参数包括：

- `warning_speed_limit_ratio`
- `hold_speed_limit_ratio`
- `emergency_speed_limit_ratio`
- `min_direction_speed`
- `reverse_escape_direction`
- `require_operator_ack_for_warning`
- `require_operator_ack_for_emergency`
- `hold_on_static_warning`
- `hold_on_combined_warning`

### 5.2 新增启动文件

文件：`src/LIO-SAM-MID360/launch/warning_controller.launch`

用途：

- 单独启动控制接口原型节点
- 通过 YAML 加载参数

## 6. `system_bringup` 接入方式

### 6.1 `run.launch`

- 新增参数：`enable_warning_controller`
- 默认值：`true`

原因：

- `run.launch` 主要用于 rosbag 回放验证，默认打开控制命令输出更方便离线联调。

### 6.2 `run_live.launch`

- 新增参数：`enable_warning_controller`
- 默认值：`false`

原因：

- 在线系统更敏感，控制接口虽然目前只发 ROS 话题，但仍然不应默认打开。

## 7. 当前限制

- 当前 `warning_controller` 仍然只基于 `WarningState` 做规则映射，没有直接读取障碍物几何细节。
- `recommended_direction` 只是基于目标自身速度反向给出的退让建议，还不是严格意义上的避障方向规划。
- 没有接入任何真实执行器、PLC 或上位机协议。
- 还没有实现动作保持时间、命令超时和人工复位状态机。

## 8. 当前建议用法

### rosbag 回放验证

```bash
cd /home/nvidia/ws_livox
roslaunch system_bringup run.launch
```

### 在线联调时显式开启

```bash
cd /home/nvidia/ws_livox
roslaunch system_bringup run_live.launch enable_segment:=true enable_warning_evaluator:=true enable_warning_controller:=true
```

### 仅查看控制输出

```bash
rostopic echo /warning/control_command
```

## 9. 结论

这一步已经把系统从“只有预警状态”推进到了“预警状态对应的控制接口原型输出”。

当前分层如下：

- `segment.cpp` 负责对象状态输出。
- `dynamicTracker.cpp` 负责动态轨迹状态输出。
- `warningEvaluator.cpp` 负责多层级风险判定。
- `warningController.cpp` 负责将预警状态映射为标准化控制命令。

这意味着后续如果要继续做控制闭环，优先级最高的工作不再是补消息，而是：

- 细化动作规则
- 引入执行确认
- 接入真实控制链路或仿真执行器