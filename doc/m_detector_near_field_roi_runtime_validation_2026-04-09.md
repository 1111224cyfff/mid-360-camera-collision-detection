# M-detector 近场 ROI 运行时验证记录（2026-04-09）

## 1. 背景

本次验证针对 `M-detector` 前新增的近场 ROI 点云分支，目标是确认下面三件事：

- 近场 ROI 裁剪实际在运行时生效。
- 裁剪只影响动态检测/避障支路，不改动 LIO-SAM 的输入链路。
- ROI 输出点云对下游仍保持原有 `cloud_registered` 的 frame 语义。

当前设计为：

- 输入话题：`/lio_sam/mapping/cloud_registered`
- 输出话题：`/perception/near_field_cloud_registered`
- 裁剪坐标系：`body_leveled`
- 裁剪规则：仅保留 $x^2 + y^2 \le 10^2$ 的点

实现位置：

- `src/M-detector/src/pointcloud_roi_filter.cpp`
- `src/M-detector/launch/detector_mid360_lio_sam.launch`
- `src/system_bringup/launch/run_dynamic_detector.launch`
- `src/system_bringup/launch/run.launch`

## 2. 本次运行命令

运行命令：

```bash
source devel/setup.bash && roslaunch system_bringup run_dynamic_detector.launch detector_rviz:=false
```

采样命令：

```bash
source devel/setup.bash && rostopic echo -n 5 /lio_sam/mapping/cloud_registered/header
source devel/setup.bash && rostopic echo -n 5 /perception/near_field_cloud_registered/header
source devel/setup.bash && rostopic echo -n 5 /lio_sam/mapping/cloud_registered/width
source devel/setup.bash && rostopic echo -n 5 /perception/near_field_cloud_registered/width
```

## 3. 运行时接线确认

`roslaunch --nodes system_bringup run_dynamic_detector.launch` 解析结果中，确认新节点已经进入图中：

- `/m_detector_near_field_roi_filter`
- `/dynfilter`
- `/m_detector_pointcloud_tf_bridge`
- `/dyn_cluster_node`
- `/dynamic_tracker`

说明当前链路已经变为：

```text
/lio_sam/mapping/cloud_registered
  -> /m_detector_near_field_roi_filter
  -> /perception/near_field_cloud_registered
  -> dynfilter
  -> /m_detector/point_out
  -> /m_detector/point_out_map
  -> /m_detector/dynamic_clusters
  -> /dynamic_tracker/track_states
```

而 LIO-SAM 输入仍保持原样，没有改动：

- `pointCloudTopic: /merged_livox_leveled`
- `imuTopic: /merged_imu_leveled`

## 4. Header / Frame 验证

### 4.1 原始 registered cloud

`/lio_sam/mapping/cloud_registered/header` 采样：

- `1775449930.100437880`, `frame_id: odom`
- `1775449930.300117970`, `frame_id: odom`
- `1775449930.600135565`, `frame_id: odom`
- `1775449930.800295591`, `frame_id: odom`
- `1775449931.000455618`, `frame_id: odom`

### 4.2 ROI cloud

`/perception/near_field_cloud_registered/header` 采样：

- `1775449960.200020313`, `frame_id: odom`
- `1775449960.600343227`, `frame_id: odom`
- `1775449960.800023317`, `frame_id: odom`
- `1775449961.100023031`, `frame_id: odom`
- `1775449961.400030851`, `frame_id: odom`

### 4.3 结论

- 两路点云都保持 `frame_id: odom`。
- 这说明 ROI 节点虽然在 `body_leveled` 中完成裁剪判断，但对外发布时仍保留了 registered cloud 的世界系语义。
- 本轮 header 采样是在不同时间窗口分别抓取，因此这里只验证“ROI 话题在持续发布，且 frame 没被改坏”，不用于逐帧 stamp 一一对齐验证。

## 5. 点数裁剪效果验证

### 5.1 原始 registered cloud 宽度采样

`/lio_sam/mapping/cloud_registered/width`：

- `12014`
- `11842`
- `11798`
- `11972`
- `11933`

平均宽度：

$$
\frac{12014 + 11842 + 11798 + 11972 + 11933}{5} = 11911.8
$$

### 5.2 ROI cloud 宽度采样

`/perception/near_field_cloud_registered/width`：

- `4960`
- `5055`
- `4967`
- `4927`
- `4765`

平均宽度：

$$
\frac{4960 + 5055 + 4967 + 4927 + 4765}{5} = 4934.8
$$

### 5.3 保留比例

保留比例：

$$
\frac{4934.8}{11911.8} \approx 41.4\%
$$

剔除比例：

$$
1 - 41.4\% = 58.6\%
$$

### 5.4 结论

- 在当前 bag 和当前轨迹下，近场 ROI 大约保留了 `41.4%` 的 registered cloud 点。
- 被剔除的点大约占 `58.6%`。
- 这说明 ROI 裁剪不是“名义接线”，而是已经实际缩减了送入动态检测支路的点云规模。

## 6. 运行现象与日志观察

本轮验证中，没有观察到 ROI 节点专属的持续告警，例如：

- `TF unavailable`
- `transformed cloud layout mismatch`
- `missing float32 x/y fields`

这说明当前实现至少满足：

- TF 查询基本可用。
- 点云字段布局能够正确读取 `x/y`。
- 输出点云布局没有立即出现结构性错误。

## 7. 当前残留问题

虽然 ROI 分支本身工作正常，但本轮 `run_dynamic_detector.launch` 仍出现过早退出，终端尾部可见：

```text
terminate called without an active exception
```

从当前证据看：

- 该问题发生在整链路退出阶段。
- 它不影响本轮完成 header 与 width 采样。
- 目前没有直接证据表明异常来自 `pointcloud_roi_filter`。

因此现阶段判断是：

- ROI 功能验证通过。
- 整链路稳定性仍需单独排查。

## 8. 本次验证结论

本轮运行时验证可以确认：

1. 近场 ROI 节点已经接入动态检测链路，并在 rosbag 回放中实际工作。
2. ROI 输出话题 `/perception/near_field_cloud_registered` 保持 `odom` frame，不会打坏下游对 registered cloud 的 frame 预期。
3. 在当前样本下，ROI 大约将送入动态检测支路的点数减少了 `58.6%`。
4. LIO-SAM 输入主链路没有被改动，仍保持 `/merged_livox_leveled` 与 `/merged_imu_leveled`。
5. 后续若继续优化，应优先排查 `run_dynamic_detector.launch` 的提前退出问题，并在此基础上继续做更长时间窗口的 ROI 效果验证。
