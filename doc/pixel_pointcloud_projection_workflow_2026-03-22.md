# 像素与点云投影流程总结（当前实现）

日期：2026-03-22  
工程：`ws_livox`  
核心代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp)
- [segment_projection.yaml](src/yolo11_ros/config/segment_projection.yaml)
- [hikrobot_camera.cpp](src/hikrobot_camera/src/hikrobot_camera.cpp)
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp)
- [run_segment.launch](src/system_bringup/launch/run_segment.launch)

> 说明：当前工程里真正实现的不是“给定任意图像像素，直接反算唯一三维点”，而是“把 LiDAR 点逐个投影到图像平面，再根据分割网络得到的像素 mask，把这些三维点归属到对应目标”。  
> 因此更准确的描述是：**点云到像素投影 + 像素实例区域到点云集合的反查**。

---

## 1. 整体目标

这个模块的目标不是生成稠密深度图，而是完成下面这件事：

1. 从相机图像中用 YOLO 分割网络得到目标实例的二维区域
2. 把点云中的每个三维点投影到图像像素坐标
3. 判断该投影像素是否落入某个实例的分割 mask
4. 如果落入，则认为这个三维点属于该实例
5. 最终得到“实例对应的点云子集”，并继续做可视化、网格占据和报警逻辑

可以把当前实现概括为：

$$
\text{LiDAR 3D points} \rightarrow \text{image pixels} \rightarrow \text{instance masks} \rightarrow \text{instance point clouds}
$$

而不是：

$$
\text{single pixel} \rightarrow \text{single 3D point}
$$

后者通常需要深度图、双目、RGB-D、稠密重建或射线求交；当前代码没有实现这条路径。

---

## 2. 输入输出与运行链路

### 2.1 图像输入

海康相机节点负责采集图像并发布 ROS 图像消息：
- 图像 frame_id 被设置为 `hikrobot_camera`
- 图像发布时间戳使用 `ros::Time::now()`

对应代码：
- [hikrobot_camera.cpp](src/hikrobot_camera/src/hikrobot_camera.cpp#L99)
- [hikrobot_camera.cpp](src/hikrobot_camera/src/hikrobot_camera.cpp#L101)

这个节点还会发布 `CameraInfo`，但当前 `segment.cpp` 里的投影实现**没有直接订阅 CameraInfo**，而是从 YAML 或 TF 获取投影所需的内外参。

### 2.2 图像坐标系重发

`camera_frame_transform_node` 会做两件事：

1. 将压缩图像重新发布到新的 topic
2. 为图像赋予新的 frame_id，并发布对应静态 TF

对应代码：
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L42)
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L60)
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L103)

当前配置文件：
- [camera_frame_transform.yaml](src/hikrobot_camera/config/camera_frame_transform.yaml)

它的作用是把原始相机帧 `hikrobot_camera` 对齐到 `hikrobot_camera_leveled`，让后续图像与 leveled 点云处在统一参考系链路下。

### 2.3 运行时使用的图像与点云 topic

系统启动时，`run_segment.launch` 默认使用：
- 图像：`/hikrobot_camera/rgb/leveled`
- 点云：`/merged_pointcloud_leveled`

对应 launch：
- [run_segment.launch](src/system_bringup/launch/run_segment.launch#L4)
- [run_segment.launch](src/system_bringup/launch/run_segment.launch#L5)

这意味着分割和投影逻辑默认工作在“整平后”的图像和点云上。

### 2.4 segment 节点订阅与同步

`segment` 节点通过 `ApproximateTime` 同步图像和点云：

- 订阅图像 topic
- 订阅点云 topic
- 使用近似时间同步器把同一时刻附近的一帧图像和一帧点云送入回调

对应代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L302)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L669)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L671)

同步窗口由参数 `sync_max_interval_sec` 控制：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L642)
- [run_segment.launch](src/system_bringup/launch/run_segment.launch#L12)

在动态场景里，这个参数非常重要。窗口过大，会造成图像和点云错帧，进而导致投影漂移。

---

## 3. 当前实现中的关键认知

### 3.1 不是像素反求三维，而是三维点投到像素

当前代码的主方向是：

$$
\mathbf{p}_{lidar} \rightarrow \mathbf{p}_{cam} \rightarrow (u, v)
$$

其中：
- $\mathbf{p}_{lidar}$ 是点云中的三维点
- $\mathbf{p}_{cam}$ 是相机坐标系中的三维点
- $(u, v)$ 是投影后的图像像素坐标

然后再用实例分割得到的像素区域去筛选点云。

### 3.2 “像素到点云”的含义其实是集合映射

在本实现里，“像素到点云”的真正含义不是：

1. 任取一个像素
2. 计算一个唯一三维坐标

而是：

1. 先有一个二维 mask 区域
2. 找出所有投影落在这个区域内的三维点
3. 这些点构成该目标的点云子集

因此输出是：
- 某个目标对应的一组点
- 该组点的中心或占据网格
- 该组点的着色点云可视化

而不是单像素深度。

---

## 4. 投影模型的数据结构

投影模型定义在：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L37)

结构体 `ProjectionModel` 包含：

1. `intrinsic`：相机内参矩阵 $K$
2. `final_rotation_matrix`：从点云参考系到相机参考系的旋转矩阵 $R$
3. `t_vec`：从点云参考系到相机参考系的平移向量 $t$
4. `flip_lidar_y`：是否在投影前把点云的 $y$ 轴取反

默认构造为：
- `intrinsic = I`
- `final_rotation_matrix = I`
- `t_vec = 0`

对应代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L43)

---

## 5. 投影参数的来源

### 5.1 YAML 备用配置

默认配置文件：
- [segment_projection.yaml](src/yolo11_ros/config/segment_projection.yaml)

该文件里定义了：

1. `intrinsic`
2. `final_rotation_matrix`
3. `t_vec`

对应位置：
- [segment_projection.yaml](src/yolo11_ros/config/segment_projection.yaml#L20)
- [segment_projection.yaml](src/yolo11_ros/config/segment_projection.yaml#L34)
- [segment_projection.yaml](src/yolo11_ros/config/segment_projection.yaml#L50)

注释里已经明确给出了计算顺序：

1. 组装点云点 $p_{lidar}$
2. 外参变换到相机坐标系：$p_{cam} = R p_{lidar} + t$
3. 用内参投影到图像平面

### 5.2 TF 优先，YAML 兜底

运行时，代码优先尝试通过 TF 动态查询图像帧与点云帧之间的变换：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L344)

在回调里，代码执行：

1. 先复制一份默认 `projection_model_`
2. 再调用 `tryBuildProjectionModelFromTf(...)`
3. 如果 TF 查询成功，使用 TF 中的旋转和平移覆盖 YAML 外参
4. 如果 TF 查询失败，退回到 YAML 中的 `final_rotation_matrix` 和 `t_vec`

对应代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L417)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L421)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L431)

因此当前系统的运行时优先级是：

1. TF 实时变换
2. YAML 静态外参

---

## 6. camera_frame_transform.yaml 在整条链中的作用

当前打开的文件：
- [camera_frame_transform.yaml](src/hikrobot_camera/config/camera_frame_transform.yaml)

这个文件本身不直接完成点投影，但它会影响 TF 链是否正确，从而间接影响投影是否正确。

它定义了：

1. 输入压缩图像 topic
2. 输出压缩图像 topic
3. 原始图像帧 `source_frame`
4. 父坐标系 `parent_frame`
5. 外参父坐标系 `extrinsic_parent_frame`
6. 输出图像帧 `output_frame`
7. 外参方向 `extrinsic_direction`
8. 4x4 外参矩阵 `extrinsic_matrix`

### 6.1 外参加载逻辑

`camera_frame_transform_node` 在初始化时读取 `extrinsic_matrix`：
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L31)

然后根据 `extrinsic_direction` 判断矩阵方向：
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L146)

若为：
- `camera_to_lidar`，则直接使用
- `lidar_to_camera`，则调用 `matrix.inv()` 取逆

对应代码：
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L158)
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L162)

### 6.2 TF 组合逻辑

如果 `extrinsic_parent_frame != parent_frame`，节点还会额外从 TF 里查一段从 `extrinsic_parent_frame` 到 `parent_frame` 的变换，并进行矩阵连乘：

$$
T_{parent \leftarrow source} = T_{parent \leftarrow extrinsic\_parent} \cdot T_{extrinsic\_parent \leftarrow source}
$$

对应代码：
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L79)
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L87)
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L91)

之后，它把该变换发布为静态 TF：
- [camera_frame_transform_node.cpp](src/hikrobot_camera/src/camera_frame_transform_node.cpp#L109)

### 6.3 为什么这会影响 segment 投影

因为 `segment.cpp` 在启用 `use_tf_for_projection` 时，会查询：

$$
T_{image\_frame \leftarrow pointcloud\_frame}
$$

如果 `camera_frame_transform_node` 没把图像帧正确挂到 TF 树上，或者挂的方向不对，`segment.cpp` 就会：

1. 查询失败
2. 退回 YAML
3. 甚至在 TF 有错误时造成投影错位

---

## 7. 单个三维点如何投影到像素

核心函数：
- [projectToImagePlane](src/yolo11_ros/src_ros/segment.cpp#L101)

### 7.1 第一步：取点云中的一个点

输入点类型为 `pcl::PointXYZI`，包含：
- `x`
- `y`
- `z`
- `intensity`

其中投影只用到 `x, y, z`。

### 7.2 第二步：根据配置决定是否翻转 y 轴

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L103)

如果 `flip_lidar_y = true`，则使用：

$$
\mathbf{p}_{lidar} =
\begin{bmatrix}
x \\
-y \\
z
\end{bmatrix}
$$

否则使用：

$$
\mathbf{p}_{lidar} =
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
$$

这一步是为了兼容某些点云坐标约定和相机投影代码的轴方向不一致问题。

### 7.3 第三步：外参变换到相机坐标系

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L105)

公式：

$$
\mathbf{p}_{cam} = R \mathbf{p}_{lidar} + t
$$

展开写成：

$$
\begin{bmatrix}
X_c \\
Y_c \\
Z_c
\end{bmatrix}
=
\begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}
\begin{bmatrix}
X_l \\
Y_l \\
Z_l
\end{bmatrix}
+
\begin{bmatrix}
t_x \\
t_y \\
t_z
\end{bmatrix}
$$

这里：
- $R$ 来自 `final_rotation_matrix`
- $t$ 来自 `t_vec`

### 7.4 第四步：剔除相机后方的点

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L106)

如果：

$$
Z_c \le 0
$$

则该点在相机后方或位于投影奇异位置，函数返回 `NaN`，后续直接丢弃。

### 7.5 第五步：通过内参投影到图像平面

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L109)

相机内参矩阵：

$$
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

先做齐次投影：

$$
\begin{bmatrix}
u' \\
v' \\
w'
\end{bmatrix}
=
K
\begin{bmatrix}
X_c \\
Y_c \\
Z_c
\end{bmatrix}
$$

再归一化得到像素坐标：

$$
u = \frac{u'}{w'}, \qquad v = \frac{v'}{w'}
$$

也就是常见形式：

$$
u = f_x \frac{X_c}{Z_c} + c_x
$$

$$
v = f_y \frac{Y_c}{Z_c} + c_y
$$

函数最终返回 `cv::Point2d(u, v)`。

---

## 8. 图像中的实例分割结果如何表示

YOLO 分割输出会被转换成内部结构 `SegmentObject`：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L30)

每个 `SegmentObject` 包含：

1. `rect`：目标框
2. `label`：类别 ID
3. `prob`：置信度
4. `boxMask`：该框范围内的二值实例 mask

转换逻辑在：
- [convertDetectionsToObjects](src/yolo11_ros/src_ros/segment.cpp#L123)

具体步骤：

1. 从检测结果 `det.bbox` 取出左上和右下坐标
2. 把 bbox 限制在图像边界内
3. 用 `det.maskMatrix` 构造整幅图的浮点 mask
4. 只截取 bbox 范围内的 ROI
5. 以阈值 0.5 将 mask 转成二值图
6. 保存到 `obj.boxMask`

对应代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L127)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L137)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L140)

因此后续判断某个投影点是否属于该目标，并不是只看它是否落在框内，而是要进一步看它是否落在该目标的二值 mask 内。

---

## 9. 三维点如何被归属到某个实例

核心处理函数：
- [drawSegmentObjects](src/yolo11_ros/src_ros/segment.cpp#L153)

### 9.1 先遍历所有点云点

主循环从这里开始：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L189)

对每个点：

1. 调用 `projectToImagePlane(...)`
2. 获得投影像素 `(x, y)`
3. 如果结果是 `NaN`，直接跳过
4. 如果超出图像边界，直接跳过

对应代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L190)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L196)

### 9.2 再遍历所有实例目标

对同一个投影点，代码会尝试判断它属于哪个实例：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L200)

具体做法：

1. 先将投影点换算到该实例 bbox 内的局部坐标
2. 判断局部坐标是否仍在 bbox 范围内
3. 如果在范围内，则访问 `obj.boxMask(relative_y, relative_x)`
4. 若该值非 0，则说明投影点落在该实例真实分割区域中
5. 把这个三维点加入 `objects_pointcloud[i]`
6. 并将 `assigned = true`

对应代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L203)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L208)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L209)

这一步的本质是：

$$
\text{if } (u,v) \in \text{instance mask}_i \Rightarrow \text{point} \in \text{object point cloud}_i
$$

### 9.3 不是框匹配，而是 mask 匹配

这一点非常关键。代码不是简单地判断“投影点落在 bbox 里就算属于物体”，而是必须满足：

1. 投影点在 bbox 内
2. 对应的 mask 像素非零

所以这是一个“框过滤 + 掩膜精筛”的过程，能比仅用 bbox 更准确地去掉背景点。

---

## 10. 为什么这能实现“从像素区域找对应点云”

因为虽然主循环是按点云点来遍历，但从集合关系上看，它等价于：

1. 图像中已有一个像素区域 $M_i$
2. 遍历所有三维点 $p_j$
3. 对每个点做投影，得到像素 $\pi(p_j)$
4. 如果 $\pi(p_j) \in M_i$，则把 $p_j$ 放入目标 $i$ 的点集

形式化写成：

$$
\mathcal{P}_i = \{ p_j \mid \pi(p_j) \in M_i \}
$$

其中：
- $\pi(\cdot)$ 是投影函数
- $M_i$ 是第 $i$ 个实例的 mask 区域
- $\mathcal{P}_i$ 是第 $i$ 个实例对应的三维点集

所以它确实实现了“从像素实例区域反查点云集合”，只是实现方式不是直接按像素扫描，而是按点扫描再筛选。

---

## 11. 回调函数中的完整执行流程

完整回调函数：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L387)

### 11.1 ROS 点云转 PCL 点云

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L389)

作用：将 `sensor_msgs::PointCloud2` 转成 `pcl::PointCloud<pcl::PointXYZI>`，方便逐点访问。

### 11.2 ROS 图像转 OpenCV 图像

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L394)

作用：把 ROS `sensor_msgs::Image` 转成 `cv::Mat`，供 TensorRT YOLO 推理使用。

### 11.3 执行实例分割推理

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L396)

作用：调用 `detector_->inference(image)` 得到检测框、类别、分割 mask 等结果。

### 11.4 将检测结果转为内部对象

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L398)

作用：把网络输出转换为 `SegmentObject` 列表，便于后续统一处理。

### 11.5 检查图像和点云的时间同步误差

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L401)

作用：通过时间戳差值判断同步是否合理。若误差过大，会打印 warning。

### 11.6 构建当前帧使用的投影模型

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L417)

作用：

1. 默认先用 YAML 加载的模型
2. 若 TF 可用，则用 TF 覆盖外参
3. 得到当前帧真正使用的 `active_projection_model`

### 11.7 绘制实例、做点投影和归属

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L436)

这一步返回：

1. `pointcloud_vec`：每个实例对应的一组点云点
2. `img_res_`：叠加分割结果的图像
3. `colored_cloud`：按是否匹配实例着色后的点云

### 11.8 进行目标网格占据分析

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L456)

流程：

1. 把每个实例点集投影到固定二维网格
2. 更新人员/挖机/吊机占据情况
3. 生成危险区域网格
4. 判断是否需要触发报警

这一步不是投影核心，但它依赖前一步已经得到的实例点云结果。

### 11.9 生成仅用于可视化的“点投影图”

代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L608)

这部分会把所有点再次投影到图像上，用小圆点绘制出来，方便在图像上观察投影对齐效果。

### 11.10 发布结果

最终发布：

1. 带分割结果的图像
2. 带全部投影点的可视化图像
3. 着色点云

对应代码：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L616)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L619)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L622)

---

## 12. 数学上真正实现的映射关系

### 12.1 点到像素的映射

投影函数为：

$$
\pi(\mathbf{p}) = \left(
f_x \frac{X_c}{Z_c} + c_x,
f_y \frac{Y_c}{Z_c} + c_y
\right)
$$

其中：

$$
\begin{bmatrix}
X_c \\
Y_c \\
Z_c
\end{bmatrix}
= R
\begin{bmatrix}
X_l \\
Y_l \\
Z_l
\end{bmatrix}
+ t
$$

### 12.2 像素区域到点集的映射

每个实例区域对应点集：

$$
\mathcal{P}_i = \{\mathbf{p} \mid \pi(\mathbf{p}) \in M_i\}
$$

所以当前实现其实是在离散点集上求逆像：

$$
\pi^{-1}(M_i) \cap \mathcal{P}_{all}
$$

其中：
- $M_i$ 是第 $i$ 个实例 mask
- $\mathcal{P}_{all}$ 是当前帧全部点云点集合

这也是“像素区域找点云”的严格数学含义。

---

## 13. 技术细节与容易忽视的点

### 13.1 CameraInfo 没有直接参与投影

虽然海康相机会发布 `CameraInfo`：
- [hikrobot_camera.cpp](src/hikrobot_camera/src/hikrobot_camera.cpp#L103)

但 `segment.cpp` 没有订阅 `CameraInfo`，也没有使用 `image_geometry::PinholeCameraModel`。  
当前投影完全依赖：

1. YAML 中的 `intrinsic`
2. TF 或 YAML 中的外参

这意味着如果相机标定更新了，而 `segment_projection.yaml` 没同步更新，就会出现投影误差。

### 13.2 时间同步误差会直接表现为投影漂移

当目标或车体在运动时，若图像与点云不是严格同一时刻，投影将产生明显横向偏移。  
代码里专门检查：
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L401)

在当前 launch 中，默认把窗口收紧到 `0.03s`：
- [run_segment.launch](src/system_bringup/launch/run_segment.launch#L12)

### 13.3 TF 正确性比 YAML 正确性更重要

因为默认 `use_tf_for_projection=true`：
- [run_segment.launch](src/system_bringup/launch/run_segment.launch#L10)
- [segment.cpp](src/yolo11_ros/src_ros/segment.cpp#L640)

所以一旦 TF 树存在错误但查询成功，系统会优先采用错误的 TF，而不是 YAML。  
这类问题通常表现为：

1. 投影整体偏移
2. 左右颠倒
3. 上下颠倒
4. 旋转角度不对

### 13.4 当前实现没有遮挡建模

代码只是检查“点投影后落在 mask 内”。它没有进一步判断：

1. 这个点是否被前景遮挡
2. 多个深度层是否发生重叠
3. 同一像素上哪个点最接近相机

也就是说，它没有构建 z-buffer 或深度竞争机制。  
因此在复杂遮挡场景中，可能会把背景点误归属到前景目标。

### 13.5 当前实现没有稠密像素深度

这不是深度图生成器。  
一个实例 mask 覆盖很多像素，但只有那些恰好有 LiDAR 点投到其上的像素位置，才会得到三维点支持。  
因此：

1. 远距离目标可能只有少量点
2. 细小目标可能没有足够点
3. 遮挡区域可能缺少点

---

## 14. 当前实现的优点与局限

### 14.1 优点

1. 实现简单直接，计算路径清晰
2. 充分利用 LiDAR 几何精度和图像分割语义信息
3. 使用 mask 而非纯 bbox，归属精度更高
4. 可直接得到每个实例的三维点集，便于后续风险分析

### 14.2 局限

1. 不是像素级稠密三维重建
2. 依赖 TF 和外参准确性
3. 依赖图像与点云时间同步质量
4. 没有遮挡消解机制
5. 对远距离稀疏点目标不够稳定

---

## 15. 实现总结

当前工程中“像素向点云投影”的真实实现可以总结为下面这条完整链路：

1. 海康相机发布图像并赋予相机 frame
2. `camera_frame_transform_node` 根据外参和 TF 关系发布 leveled 图像帧
3. `segment` 节点同步图像与点云
4. YOLO 分割得到每个目标的 bbox 和 mask
5. 对点云中的每个三维点执行外参变换和相机投影
6. 判断投影像素是否落入某个实例 mask
7. 若落入，则把该点归到该实例点云中
8. 再基于实例点云做着色、网格占据和报警逻辑

因此，当前实现的本质是：

$$
\text{3D point projection} + \text{instance mask filtering}
$$

也可以更准确地表述为：

$$
\text{image semantics guide point cloud selection}
$$

它不是从单个像素反求三维点，而是用像素语义去筛选属于该目标的点云集合。

---

## 16. 后续若要继续增强，可考虑的方向

1. 使用 `CameraInfo` 直接驱动内参，而不是手写 YAML
2. 对投影结果加入深度竞争或 z-buffer 机制，减少遮挡误匹配
3. 以实例点集为基础做三维包围盒拟合
4. 对远距离目标增加多帧点云累积，提高稀疏目标稳定性
5. 若需要“真正的像素到三维点”，则额外建立稠密深度图或像素最近点索引