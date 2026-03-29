# 塔吊吊钩场景：一次性 IMU 重力水平化（Livox MID360 / LIO-SAM）导出记录

日期：2026-01-21

## 1. 需求概述
- 传感器安装在塔吊吊钩上，LiDAR 视线/姿态相对地面存在固定倾斜（示例：Z 轴指向地面，偏离垂直约 38°）。
- 目标：将点云“水平化”（使地面与实际地面平行），并且该变换应为“一次性”标定固定，不进行持续姿态纠正，以避免吊钩摆动/动态加速度对 LIO-SAM 输入造成扰动。
- 现有输入：
  - `/merged_livox`：Livox CustomMsg 合并点云
  - `/merged_imu`：IMU 数据（sensor_msgs/Imu），400Hz

## 2. 方案核心
- 利用 IMU 在静止时段的线加速度平均值方向来估计“上方向”。
  - 加速度计在静止时测得的 proper acceleration 方向近似为 `+g`（指向“上”），通过一段时间窗口平均可降低噪声。
- 仅在启动后的标定窗口内（默认 2 秒）进行采样并计算一次固定旋转矩阵 `R_level`：
  - 将观测到的上方向 `up` 旋转到目标 `Z_up = [0,0,1]`。
- 标定完成后：
  - 对点云中的每个点 `(x,y,z)` 应用固定旋转 `p' = R_level * p`。
  - 同样对 IMU 的向量量（`linear_acceleration` / `angular_velocity`）应用相同旋转，确保与点云处于同一“leveled”坐标意义下。
- 发布门控：默认 **标定完成前不发布** leveled 话题（避免下游误用未标定数据）。

## 3. 新增/变更的话题
### 3.1 输出（新增）
- `/merged_pointcloud_leveled`（sensor_msgs/PointCloud2）
  - 来自 `/merged_pointcloud`（PointCloud2），点坐标做固定旋转。
- `/merged_livox_leveled`（livox_ros_driver2/CustomMsg）
  - 来自 `/merged_livox`（CustomMsg），每个点做固定旋转。
- `/merged_imu_leveled`（sensor_msgs/Imu）
  - 来自 `/merged_imu`，`linear_acceleration` 和 `angular_velocity` 做固定旋转；若 orientation 四元数近似有效，则也做固定旋转。

### 3.2 输入（已有）
- `/merged_imu`：用于一次性标定与（可选）leveled IMU 输出
- `/merged_pointcloud`：用于 leveled PointCloud2 输出
- `/merged_livox`：用于 leveled CustomMsg 输出

## 4. 与 LIO-SAM 的对接结论
- **推荐** LIO-SAM 同时使用：
  - 点云：`/merged_livox_leveled`
  - IMU：`/merged_imu_leveled`
- 不建议“点云 leveled，但 IMU 仍用 `/merged_imu` 原始未旋转”的组合：
  - 这会导致点云和 IMU 的坐标意义不一致，相当于引入了额外外参旋转但没有在系统中显式建模，可能破坏重力方向/预积分一致性。

## 5. 代码与配置改动位置
### 5.1 新增节点（livox_merge 包内）
- 源码：`src/Livox_merge/src/merged_pointcloud_leveler_node.cpp`
  - 功能：一次性 IMU 标定 + leveled PointCloud2/CustomMsg/IMU 输出 + 发布门控

### 5.2 CMake
- `src/Livox_merge/CMakeLists.txt`
  - 新增可执行：`merged_pointcloud_leveler_node`

### 5.3 Launch
- `src/Livox_merge/launch/merged_pointcloud_leveler.launch`
  - 参数：
    - `imu_topic`（默认 `/merged_imu`）
    - `imu_output_topic`（默认 `/merged_imu_leveled`）
    - `cloud_topic`（默认 `/merged_pointcloud`）
    - `output_topic`（默认 `/merged_pointcloud_leveled`）
    - `livox_custom_topic`（默认 `/merged_livox`）
    - `livox_custom_output_topic`（默认 `/merged_livox_leveled`）
    - `calib_duration_sec`（默认 `2.0`）
    - `min_imu_samples`（默认 `800`，对应 2s@400Hz）
    - `publish_before_calibrated`（默认 `false`，即标定前不输出）

### 5.4 Bringup 集成
- `src/system_bringup/launch/run.launch`
  - 已 include 上述 leveled 节点 launch，使其跟随 bringup 启动。

### 5.5 LIO-SAM 配置
- `src/LIO-SAM-MID360/config/paramsLivoxIMU.yaml`
  - 已将：
    - `lio_sam.pointCloudTopic` -> `/merged_livox_leveled`
    - `lio_sam.imuTopic` -> `/merged_imu_leveled`

## 6. 使用说明（验证）
1) 编译：
- 在工作区根目录运行 `catkin_make -DCMAKE_BUILD_TYPE=Release`

2) 运行：
- `roslaunch system_bringup run.launch`

3) 观察：
- RViz 订阅：
  - `/merged_pointcloud_leveled`（PointCloud2）
  - 或 `/merged_livox_leveled`（若你有相应显示/转换链路）
- 节点启动后会在约 2 秒后打印一次性标定结果（平均加速度、up 向量、旋转矩阵/四元数）。

## 7. 备注与可选增强
- 当前实现为“标定前不发布 leveled 输出”。如果希望 LIO-SAM 不丢 IMU（例如 bag 播放开始就有 IMU），可增强为：
  - 标定期缓存 IMU（ring buffer），标定完成后补发缓存再实时输出。
