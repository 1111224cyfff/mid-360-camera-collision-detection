# 动态点识别流程总结（当前实现）

日期：2026-01-23  
工程：`ws_livox/src/LIO-SAM-MID360`  
核心代码：
- [mapOptmization.cpp](src/LIO-SAM-MID360/src/mapOptmization.cpp)
- [utility.h](src/LIO-SAM-MID360/include/utility.h)
- [paramsLivoxIMU.yaml](src/LIO-SAM-MID360/config/paramsLivoxIMU.yaml)

> 说明：这里的“残差”不是 LIO-SAM Scan-to-Map 优化里点到线/点到面 LM 残差，而是 **“点到局部地图最近邻距离（nnDist）”** 作为一个 *residual proxy*（近似残差指标）。

---

## 1. 输入/输出（ROS 话题层面）

### 1.1 输入
动态点识别发生在 Mapping 节点（`mapOptimization`）发布帧数据时，它会使用 **去畸变点云** 并把它变换到 `odometryFrame`：
- 输入点云：`cloudInfo.cloud_deskewed`（来自 `lio_sam/feature/cloud_info`）
- 变换：使用当前 `transformTobeMapped`，将点云变到 `odometryFrame`

### 1.2 输出（本模块相关）
由 Mapping 节点发布（frame_id = `odometryFrame`）：
- `/lio_sam/mapping/cloud_dynamic`：高置信度动态点（RViz 通常标红）
- `/lio_sam/mapping/cloud_dynamic_low`：低置信度动态点（RViz 通常标灰）
- `/lio_sam/mapping/cloud_dynamic_clustered`：聚类后的动态点云（**点的 intensity = clusterId**，便于按簇上色/调试）
- `/lio_sam/mapping/dynamic_clusters`：簇的包围盒 Marker（可选）
- `/lio_sam/mapping/dynamic_cluster_centers`：簇中心 PoseArray（可选）

> 计算触发条件：只有当 `dynamic_point_detection/enabled` 打开，且上面这些 topic 至少一个有订阅者时，才会在 `publishFrames()` 里计算动态点（节省算力）。

---

## 2. 参考地图（“对比的局部地图”是什么）

动态判别不是拿“全局地图”，而是拿 **局部子地图**：
- 局部地图由两部分拼起来：
  - `laserCloudCornerFromMapDS`（局部角点特征，下采样后）
  - `laserCloudSurfFromMapDS`（局部平面特征，下采样后）
- 在代码里通过 `buildLocalMapCloud()` 合并为 `mapLocal`

这些 `FromMapDS` 本质来源于：
- 当前位姿附近的一段关键帧集合（surrounding keyframes）拼接
- 再做体素下采样（DS）

所以你的“对比对象”是：**“当前帧(已配准) vs 当前位姿附近的局部特征地图”**。

---

## 3. 动态点判别（最近邻距离 + 邻域支持）

核心函数：`extractDynamicWithConfidence(...)`。

### 3.1 点云准备
1. 拿到已变换到 `odometryFrame` 的 `registeredCloud`
2. （可选）先对 `registeredCloud` 做体素下采样得到 `testCloud`：
   - 参数：`dynamic_point_detection/downsample_leaf_size`
   - 目的：减少逐点 KNN 查询开销
3. 为局部地图 `mapLocal` 建 KD-Tree：`kdtreeMap.setInputCloud(mapLocal)`

### 3.2 对每个点做“动态性”判断
对 `testCloud` 中每个点 `pt`：

**(A) 距离/范围裁剪**
- 计算量程 $r = \sqrt{x^2+y^2+z^2}$
- 若 `max_range > 0` 且 $r > max\_range$，跳过（不参与动态判断）

**(B) 最近邻距离（nnDist）**
- 在 `mapLocal` 上做 KNN（k = `knn_k`）
- 取最近邻的距离：$nnDist = \sqrt{nnSqDist[0]}$

**(C) 邻域支持（neighbor support）**
为了避免“局部地图稀疏导致误判”，增加一个“邻居数量”条件：
- 若 `neighbor_radius > 0`：做半径搜索 `radiusSearch(pt, neighbor_radius)` 得到 `neighborCount`
- 否则：用 KNN 的 found 数作为 `neighborCount`
- 要求：
  - 若 `min_neighbors <= 0`：不限制
  - 否则：必须 `neighborCount >= min_neighbors`

**(D) 阈值模型（随距离放宽）**
定义动态判别阈值：

$$thresh = knn\_max\_dist\_base + knn\_max\_dist\_scale \cdot r$$

判别为动态的条件：

$$isDynamic = enoughNeighbors \land (nnDist > thresh)$$

直观解释：
- 点和局部地图最近邻“差太远” → 更像是新出现的物体/变化部分
- 距离越远允许更大的误差（scale * r）

---

## 4. 置信度（把“动态程度”变成 0..1 的分数）

当某个点已被判为动态（`isDynamic == true`）后，计算它的置信度：

$$conf = clamp\left(\frac{nnDist - thresh}{confidence\_scale}, 0, 1\right)$$

- `confidence_scale`：把“超出阈值的距离”映射成 0..1 的尺度（单位 m）
- `confidence_threshold`：把动态点再分成高/低置信度两类

分流规则：
- `conf >= confidence_threshold` → 进 `/cloud_dynamic`（高置信度）
- `conf <  confidence_threshold` → 进 `/cloud_dynamic_low`（低置信度）

> 注意：目前实现是 **先用 nnDist>thresh 判是否动态**，只对动态点计算 conf；非动态点不会出现在动态相关输出中。

---

## 5. 聚类（Euclidean clustering）与“离散噪点过滤”

### 5.1 聚类对象
聚类输入是 `rawDynamic`（所有动态点，不区分高/低置信度），并且与其一一对应保存了 `rawConfidence`。

### 5.2 聚类算法
当 `clustering_enabled: true` 时：
- 使用 `pcl::EuclideanClusterExtraction` 在 `rawDynamic` 上做欧式聚类
- 关键参数：
  - `cluster_tolerance`：点间连通距离阈值（m）
  - `cluster_min_size` / `cluster_max_size`

当 `clustering_enabled: false` 时：
- 把所有动态点当成一个“簇”（便于复用后续逻辑）

### 5.3 过滤策略（filter_by_cluster）
当 `filter_by_cluster: true` 时：
- `dynamicHighOut` 和 `dynamicLowOut` 会被清空并重新填充
- **只保留属于某个簇的动态点**

作用：
- 用聚类来抑制“零星散点”的动态误检（你提到的“过滤离散噪点”）

> 小提示：如果你发现启用 `filter_by_cluster` 后动态点几乎没了，通常是 `cluster_min_size` 太大、或 `cluster_tolerance` 太小，导致无法形成足够大的簇。

### 5.4 聚类可视化输出（cloud_dynamic_clustered）
- `dynamicClusteredOut` 汇总每个簇的点
- 写入规则：对簇内每个点复制一份，并设置：
  - `p.intensity = clusterId`

因此 RViz 可以用 Intensity 上色，从而直观看到不同簇的分割效果。

同时可选输出：
- `dynamic_clusters`：每个 cluster 一个 CUBE marker（红色半透明包围盒）
- `dynamic_cluster_centers`：每个 cluster 一个中心点 Pose

---

## 6. 与建图的关系：动态点是否参与地图更新

参数：`dynamic_point_detection/exclude_from_map`（对应 `dynamicExcludeFromMap`）
- 目的：将判定为动态的点从地图/关键帧中剔除，减少动态物体“写入地图”的概率
- 风险：如果动态判别过于激进，会把静态结构（例如你提到的“新墙面”）误当动态，从而导致地图缺失/稀疏

> 你当前的配置 `exclude_from_map: true`，更偏向“把动态抑制掉”的建图策略。

---

## 7. 关键参数清单（你现在最常调的那些）

对应 [paramsLivoxIMU.yaml](src/LIO-SAM-MID360/config/paramsLivoxIMU.yaml) / `dynamic_point_detection:`：

- `enabled`：总开关
- `knn_k`：KNN 的 k
- `knn_max_dist_base`：近距离阈值基线（m）
- `knn_max_dist_scale`：随距离放宽系数（m/m）
- `neighbor_radius`：邻域半径（m）
- `min_neighbors`：邻域最少点数
- `downsample_leaf_size`：测试点云下采样 leaf（m）
- `max_range`：只在此范围内做动态判断（m）
- `confidence_threshold`：高低置信度分界（0..1）
- `confidence_scale`：把 (nnDist-thresh) 拉伸到 0..1 的尺度（m）
- `clustering_enabled`：是否聚类
- `filter_by_cluster`：是否用聚类过滤离散点
- `cluster_tolerance` / `cluster_min_size` / `cluster_max_size`
- `publish_markers` / `publish_centers`

---

## 8. 伪代码（按当前实现抽象）

```text
registeredCloud = transform(cloud_deskewed, odometryFrame)
mapLocal = cornerFromMapDS + surfFromMapDS

testCloud = registeredCloud
if downsample_leaf_size > 0:
  testCloud = VoxelGrid(registeredCloud)

kdtreeMap = KDTree(mapLocal)

rawDynamic = []
rawConfidence = []
for pt in testCloud:
  r = range(pt)
  if max_range>0 and r>max_range: continue

  nnDist = nearest_neighbor_distance(kdtreeMap, pt, knn_k)
  neighborCount = radiusSearch(kdtreeMap, pt, neighbor_radius) or knn_found

  thresh = knn_max_dist_base + knn_max_dist_scale * r
  if neighborCount < min_neighbors: continue
  if nnDist <= thresh: continue

  conf = clamp((nnDist - thresh) / confidence_scale, 0..1)
  rawDynamic.push(pt)
  rawConfidence.push(conf)
  if conf >= confidence_threshold: dynamicHigh.push(pt)
  else: dynamicLow.push(pt)

clusters = EuclideanCluster(rawDynamic) or {all points}

if filter_by_cluster:
  dynamicHigh.clear(); dynamicLow.clear()
  for each cluster c:
    for idx in c.indices:
      conf = rawConfidence[idx]
      if conf >= confidence_threshold: dynamicHigh.push(rawDynamic[idx])
      else: dynamicLow.push(rawDynamic[idx])

for each clusterId, cluster c:
  for idx in c.indices:
    p = rawDynamic[idx]
    p.intensity = clusterId
    dynamicClustered.push(p)
  optionally publish marker + center

publish dynamicHigh/dynamicLow/dynamicClustered (+markers/centers)
```

---

## 9. 你现在这套流程的“直观效果”

- `/cloud_dynamic`：更“确定”的变化点（nnDist 明显超阈值）
- `/cloud_dynamic_low`：刚超阈值的点（更偏不确定/可疑）
- `/cloud_dynamic_clustered`：把动态点按空间连通性分簇，并用 `intensity=clusterId` 标记
- `filter_by_cluster=true`：能有效压掉“散点噪声”，但可能也会压掉小物体/稀疏目标（取决于 min_size/tolerance）

