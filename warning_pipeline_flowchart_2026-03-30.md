# 多层级预警机制流程图（论文/汇报版）

## 1. 总体流程图

```mermaid
flowchart TD
    A[YOLO26分割图像输入] --> B[实例分割结果]
    C[Leveled点云输入] --> D[3D点投影到2D图像]
    B --> D
    D --> E[按mask回查实例点云集合]
    E --> F[构建吊装目标对象状态<br/>质心 尺寸 包络半径 最近距离 点数]

    G[动态点检测与聚类] --> H[Dynamic Tracker]
    H --> I[动态轨迹状态<br/>ID 位置 速度 预测轨迹]

    J[LIO-SAM局部地图 map_local] --> K[静态净空评估]
    F --> K

    F --> L[目标速度估计]
    L --> M[动态风险评估]
    I --> M

    K --> N[静态风险等级]
    M --> O[动态风险等级]
    O --> P{是否相向运动升级}
    P --> Q[动态等级修正]

    N --> R[融合判级<br/>取静态与动态最大等级]
    Q --> R

    R --> S[迟滞状态机<br/>升级立即生效 降级延时1秒]
    S --> T[WarningState输出<br/>warning/state]
    T --> U[RViz预警可视化<br/>warning/markers]
    T --> V[Warning Controller]
    V --> W[ControlCommand输出<br/>warning/control_command]
```

## 2. 预警判级细化流程图

```mermaid
flowchart TD
    A[接收对象状态 object_states] --> B{对象状态是否有效}
    B -- 否 --> Z1[输出 clear<br/>reason=waiting_for_inputs]
    B -- 是 --> C[筛选监控目标<br/>crane exca-arm exca-body]

    C --> D{是否找到满足条件目标}
    D -- 否 --> Z2[输出 clear<br/>reason=no_monitored_object]
    D -- 是 --> E[坐标变换到 map]

    E --> F{TF是否成功}
    F -- 否 --> Z3[输出 clear<br/>reason=object_tf_failed]
    F -- 是 --> G[估计目标速度]

    G --> H[静态净空计算<br/>目标到map_local最近邻距离减包络半径]
    G --> I[动态风险计算<br/>当前净空 相对速度 TTC 预测净空]

    H --> J{静态净空阈值}
    J -- d <= 0.8m --> J3[静态等级 Emergency]
    J -- 0.8m < d <= 1.5m --> J2[静态等级 Warning]
    J -- 1.5m < d <= 2.5m --> J1[静态等级 Notice]
    J -- d > 2.5m --> J0[静态等级 None]

    I --> K{动态阈值或TTC阈值}
    K -- clearance<=0.6m 或 TTC<=1.0s --> K3[动态等级 Emergency]
    K -- clearance<=1.2m 或 TTC<=2.0s --> K2[动态等级 Warning]
    K -- clearance<=2.0m 或 TTC<=3.0s --> K1[动态等级 Notice]
    K -- 否则 --> K0[动态等级 None]

    K3 --> L
    K2 --> L
    K1 --> L
    K0 --> L

    L{是否相向运动}
    L -- 是 --> M[动态等级上调一级<br/>最高不超过Emergency]
    L -- 否 --> N[保持当前动态等级]
    M --> O[融合静态与动态等级]
    N --> O[融合静态与动态等级]

    J3 --> O
    J2 --> O
    J1 --> O
    J0 --> O

    O --> P[desired_level]
    P --> Q[迟滞状态机]
    Q --> R[active_level]
    R --> S[发布 WarningState]
```

## 3. 控制接口映射流程图

```mermaid
flowchart TD
    A[接收 WarningState] --> B{system_ready}
    B -- 否 --> Z[输出 ACTION_CLEAR]
    B -- 是 --> C{active_level}

    C -- None --> D[ACTION_CLEAR<br/>不限速]
    C -- Notice --> E[ACTION_OBSERVE<br/>仅提示]
    C -- Warning --> F[ACTION_LIMIT_MOTION<br/>默认限速0.35]
    C -- Emergency --> G[ACTION_EMERGENCY_STOP<br/>限速0 保持位=true 急停位=true]

    F --> H{是否需要建议退让方向}
    G --> H
    H -- 目标速度较小 --> I[recommended_direction = 0]
    H -- 目标速度足够大 --> J[recommended_direction = 当前速度反方向]

    D --> K[发布 ControlCommand]
    E --> K
    I --> K
    J --> K
```

## 4. 图中对应的真实代码位置

- 对象状态生成：`src/yolo26_ros/src_ros/segment.cpp`
- 动态轨迹状态：`src/LIO-SAM-MID360/src/dynamicTracker.cpp`
- 风险融合与状态机：`src/LIO-SAM-MID360/src/warningEvaluator.cpp`
- 控制命令映射：`src/LIO-SAM-MID360/src/warningController.cpp`
- 风险阈值配置：`src/LIO-SAM-MID360/config/warning_evaluator.yaml`
- 控制映射配置：`src/LIO-SAM-MID360/config/warning_controller.yaml`

## 5. 论文/汇报使用建议

### 5.1 论文正文建议用法

- 图1可作为“系统总体技术路线图”。
- 图2可作为“预警判级核心流程图”。
- 图3可作为“预警到控制接口映射流程图”。

### 5.2 口头汇报建议讲法

可按以下顺序讲：

1. 先讲感知输入不是单一图像，而是图像分割、点云和动态轨迹三路信息。
2. 再讲风险不是只看瞬时距离，而是静态净空加动态趋势联合评估。
3. 最后讲系统输出不仅有预警等级，还有控制接口原型输出。
