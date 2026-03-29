# 双 Mid-360 PTP 原理、流程、问题修正与现场验证总结

日期：2026-03-24

## 1. 文档目的

本文用于系统记录当前工作区中双 Mid-360 接入 `eth0` 网络后的 PTP 设计、启动流程、之前排查中发现的错误、已完成的修正，以及 2026-03-24 这次现场验证得到的直接证据。

本文重点回答以下问题：

- 当前系统中 PTP 的工作原理是什么
- `system_bringup` 中 PTP 的启动链路是什么
- 之前实现里有哪些关键错误或误判
- 已经做了哪些修正
- 现在如何确认两台雷达确实在使用 PTP/gPTP

---

## 2. 系统拓扑与角色

当前现场状态可以概括为：

- 主机网口：`eth0`
- 主机地址：`192.168.1.200`
- 雷达 1：`192.168.1.125`
- 雷达 2：`192.168.1.148`
- 两台 Mid-360 与主机处于同一二层网络
- `collect_data_with_ptp.launch` 正在运行

当前拓扑下，主机侧 `ptp4l` 在 `eth0` 上运行，并作为当前 PTP 域中的 grandmaster；两台 Mid-360 在网络中作为从设备参与 PTP/gPTP 时间同步。

---

## 3. PTP 工作原理

### 3.1 Livox 的 PTP/gPTP 同步逻辑

根据 Livox 官方时间同步文档：

- Livox 设备在检测到网络中存在有效的 PTP/gPTP 主时钟后，会自动同步自己的设备时间
- 对 Livox 而言，`timestamp_type = 1` 表示设备正在使用 `PTP/gPTP`
- 如果网络中存在 `Sync`、`Follow_Up`，并且主从状态建立成功，则 Livox 点云和 IMU 的设备时间会跟随 PTP 域

也就是说，判断 Livox 是否真的进入 PTP，不应只看主机是否启动了 `ptp4l`，而应进一步确认：

- 网络中是否存在有效 PTP 报文
- 雷达是否参与主从交互
- 雷达原始数据包中的 `time_type/timestamp_type` 是否已经切换到 `1`

### 3.2 主机系统时间与 PHC 不是同一个概念

主机中至少存在两类时钟：

1. 网卡的 PTP Hardware Clock，通常是 `/dev/ptp0`
2. Linux 系统时间 `CLOCK_REALTIME`

二者不是天然相同。

因此需要区分两个问题：

- 雷达是否在用 PTP 时间
- 主机系统时间是否与当前 PTP 参考保持一致

这也是为什么系统里除了 `ptp4l` 之外，还需要考虑 `phc2sys`。

### 3.3 Livox 驱动中的时间戳处理逻辑

`livox_ros_driver2` 的实现里，原始以太网包中的 `time_type` 决定驱动如何生成 ROS 时间。

关键定义如下：

- 在 `src/livox_ros_driver2/src/comm/comm.h` 中：
  - `kTimestampTypeNoSync = 0`
  - `kTimestampTypeGptpOrPtp = 1`
  - `kTimestampTypeGps = 2`

- 在 `src/livox_ros_driver2/src/comm/pub_handler.cpp` 中：
  - 若 `timestamp_type == 1` 或 `2`，驱动直接使用包内设备时间戳
  - 若 `timestamp_type == 0`，驱动退回到主机当前时间

这意味着：

- 仅看 ROS 中的 `header.stamp` 是否像 epoch 时间，并不足以证明 PTP 已启用
- 必须进一步确认 Livox 原始包中的 `time_type` 是否为 `1`

---

## 4. 当前 PTP 启动流程

### 4.1 启动入口

当前入口为：

- `src/system_bringup/launch/collect_data_with_ptp.launch`

其中关键默认参数为：

- `enable_ptp_sync := true`
- `ptp_primary_iface := eth0`
- `ptp_primary_mode := auto`
- `ptp_sync_system_clock := true`
- `ptp_clock_sync_direction := system-to-phc`

该入口会包含：

- `src/system_bringup/launch/ptp_sync.launch`

### 4.2 ptp_sync.launch 的职责

`ptp_sync.launch` 负责启动：

- `src/system_bringup/scripts/ptp_time_sync.sh`

该脚本负责：

1. 检查接口是否存在、链路是否有 carrier
2. 检查 `ethtool -T` 能力，判断 `eth0` 是否具备 PHC
3. 在 `auto` 模式下自动选择 `hardware` 或 `software`
4. 启动 `ptp4l`
5. 若允许同步系统时间，则启动 `phc2sys`
6. 在 `roslaunch` 退出时清理 `ptp4l/phc2sys`

### 4.3 当前实际运行方式

本次现场中看到的实际运行进程为：

```text
sudo -n /usr/local/sbin/ptp4l -H -i eth0 -m
sudo -n /usr/local/sbin/phc2sys -s CLOCK_REALTIME -c eth0 -O 0 -m
```

这表明：

- `ptp4l` 以硬件时间戳模式运行在 `eth0`
- `phc2sys` 当前采用的是 `system-to-phc` 方向

---

## 5. 之前的错误、误判与风险

### 5.1 原先对 `phc2sys` 方向的理解不适合当前拓扑

之前一个核心假设是：

```text
CLOCK_REALTIME <- /dev/ptp0
```

也就是默认采用：

```bash
phc2sys -s /dev/ptp0 -c CLOCK_REALTIME -w -m
```

这个方向在“主机要跟随某个外部 PHC/PTP 参考”的场景下是成立的，但对当前这个拓扑并不总是合适。

当前现场拓扑是：

- 主机在 `eth0` 上自己成为当前 PTP grandmaster
- 两台雷达跟随主机这一侧的时间域

在这个场景中，Livox 官方文档给出的主机主时钟示例更接近：

```bash
phc2sys -s CLOCK_REALTIME -c eth0 -O 0
```

也就是：

```text
eth0 PHC <- CLOCK_REALTIME
```

如果在主机已经是 grandmaster 的情况下反过来强行把系统时间拉向 PHC，就可能在系统一开始存在较大偏差时，对主机系统时间造成明显扰动。

### 5.2 原脚本默认会把系统时间强拉向 PHC

在修正之前，`ptp_time_sync.sh` 默认行为等价于：

```bash
phc2sys -s /dev/ptp0 -c CLOCK_REALTIME -w -m
```

在实际复现中曾观察到：

- `phc2sys` 起步时出现了大约 37 秒量级的 offset
- 它持续尝试把 `CLOCK_REALTIME` 拉向 PHC

这会带来两个风险：

1. 主机系统时间发生明显跳变或快速调节
2. 依赖系统时间的 ROS 节点、USB 相机、日志与录包时间可能被扰动

### 5.3 之前不能仅靠 ROS 话题时间戳判断是否启用 PTP

之前容易产生的误判是：

- 看到 `header.stamp` 是纳秒级 epoch 时间
- 或者两台雷达时间戳看起来接近
- 就推断 PTP 已经工作

实际上这并不充分。

因为 `livox_ros_driver2` 在 `time_type = 0` 时会回退到主机当前时间补戳，所以：

- ROS 消息看起来“时间正常”
- 不等于 Livox 设备真的已经切换到 PTP

真正可靠的证据必须回到 Livox 原始网络包头部中的 `time_type`。

### 5.4 旧版诊断脚本存在两个问题

之前的 `scripts/check_ptp.sh` 有两个会影响排查效率的问题：

1. `ss` 查询阶段会直接调用 `sudo`，在无免密时可能卡住
2. `pmc` 查询里使用了错误的命令名 `GET TIME_PROPERTIES_DS`

这会导致：

- 诊断脚本看起来“没反应”或中途卡住
- `pmc` 无法返回完整的 PTP 数据集信息

---

## 6. 已完成的改进

### 6.1 在 PTP 启动脚本中显式引入同步方向参数

当前 `src/system_bringup/scripts/ptp_time_sync.sh` 新增了：

- `--clock-sync-direction system-to-phc|phc-to-system`

并将默认值调整为：

- `system-to-phc`

这使得脚本不再把 `phc2sys` 的方向写死，而是可以根据网络拓扑选择：

- `system-to-phc`：当前主机自己作为 PTP master 时更合适
- `phc-to-system`：未来若主机需要跟随外部 grandmaster，可显式切换

### 6.2 让 roslaunch 层可以透传同步方向

以下文件已支持透传该参数：

- `src/system_bringup/launch/ptp_sync.launch`
- `src/system_bringup/launch/collect_data_with_ptp.launch`

因此在线启动时无需手改脚本，只要通过 launch 参数即可切换策略。

### 6.3 修复 check_ptp.sh 的阻塞与命令问题

当前诊断脚本已修正为：

1. 若没有 `sudo -n` 能力，则退回非 sudo 的 `ss` 查询，不再卡死
2. `pmc` 命令更正为：

```bash
GET TIME_PROPERTIES_DATA_SET
```

这使得脚本更适合在现场快速执行。

---

## 7. 2026-03-24 双雷达现场验证过程

### 7.1 验证目标

本次验证的目标不是只确认“主机跑了 `ptp4l`”，而是确认以下四件事：

1. 双雷达是否都在线并在发布数据
2. 主机 PTP 链路是否实际工作
3. 两台雷达是否都参与 PTP 报文交互
4. 两台雷达的 Livox 原始包 `time_type` 是否都已经切换为 `1`

### 7.2 验证步骤

#### 步骤 1：检查进程

检查结果显示以下关键进程在运行：

```text
roslaunch system_bringup collect_data_with_ptp.launch
/usr/local/sbin/ptp4l -H -i eth0 -m
/usr/local/sbin/phc2sys -s CLOCK_REALTIME -c eth0 -O 0 -m
livox_ros_driver2_node
```

说明：

- PTP 启动链路已被带起
- Livox 驱动正在运行

#### 步骤 2：检查 ROS 话题

当前在线话题包括：

- `/livox/lidar_192_168_1_125`
- `/livox/lidar_192_168_1_148`
- `/livox/imu_192_168_1_125`
- `/livox/imu_192_168_1_148`

说明：

- 两台雷达与各自 IMU 数据都在发布

#### 步骤 3：检查发布频率

`rostopic hz` 结果显示：

- `/livox/lidar_192_168_1_125` 约 `10.45 Hz`
- `/livox/lidar_192_168_1_148` 约 `10.46 Hz`

说明：

- 两路点云都在稳定输出
- 没有出现一台正常、一台掉线的情况

#### 步骤 4：运行 PTP 检查脚本

运行：

```bash
scripts/check_ptp.sh -i eth0 --no-capture --pmc
```

得到关键结果：

```text
PTP Hardware Clock: 0
stepsRemoved     0
offsetFromMaster 0.0
meanPathDelay    0.0
grandmasterIdentity 001114.fffe.173441
currentUtcOffset 37
```

说明：

- `eth0` 支持硬件时间戳并存在 PHC
- 当前 PTP 域已建立
- grandmaster 身份是当前主机侧时钟

#### 步骤 5：抓取 PTP 报文

在 `eth0` 上抓取 UDP 319/320 报文后，观察到：

- `192.168.1.200 -> 224.0.1.129` 的 PTP 报文
- `192.168.1.125 -> 224.0.1.129` 的 UDP 319 报文
- `192.168.1.148 -> 224.0.1.129` 的 UDP 319 报文

说明：

- 主机在发 PTP 报文
- 两台雷达也都参与了 PTP 事件报文交互

#### 步骤 6：抓取两台雷达的 Livox 原始 UDP 数据包

抓取端口：

- 源地址 `192.168.1.125`，源端口 `56300`
- 源地址 `192.168.1.148`，源端口 `56300`

随后对照 Livox SDK 头文件 `/usr/local/include/livox_lidar_def.h` 中的结构定义：

```c
typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;
  uint8_t timestamp[8];
  uint8_t data[1];
} LivoxLidarEthernetPacket;
```

对抓包内容解读后得到：

- `192.168.1.125` 发来的原始点云包中，`time_type = 1`
- `192.168.1.148` 发来的原始点云包中，`time_type = 1`

而在 Livox 驱动源码中：

- `1` 被定义为 `gPTP or PTP sync mode`

因此可以直接得出结论：

- 两台 Mid-360 都已经进入 PTP/gPTP 时间同步模式

---

## 8. 为什么这次结论是可靠的

这次的结论不是基于单一现象，而是基于四层证据叠加：

1. **进程层**
   - `ptp4l/phc2sys` 正在运行

2. **ROS 层**
   - 双雷达、双 IMU 话题都在稳定发布

3. **PTP 管理层**
   - `pmc` 能读到当前 grandmaster 数据集

4. **协议原始包层**
   - 两台雷达的原始 Livox 数据包 `time_type` 都是 `1`

其中最关键的是第 4 点，因为它直接来源于 Livox 原始协议头，而不是 ROS 转发后的二次结果。

---

## 9. 相机与雷达时间关系的离线量化结果

### 9.1 分析背景

在确认双雷达已经进入 PTP/gPTP 模式之后，仍然需要回答另一个工程上更实际的问题：

- 相机没有直接使用 PTP，而是使用系统时间打戳，这是否意味着它已经和雷达“同步”

当前系统中：

- 双雷达使用 PTP/gPTP 设备时间
- Hikrobot 相机使用 ROS 消息头中的系统时间

因此，相机与雷达之间更准确的说法不是“严格硬同步”，而是“是否处于同一时间基准附近”。

### 9.2 分析方法

本次没有直接用在线 Python 订阅结果作为最终依据，原因是现场对 `livox_ros_driver2/CustomMsg` 的在线 Python 订阅出现了收包异常；为避免这个环境噪声影响结论，采用了更稳定的离线方法：

1. 使用当前录制的 bag 文件：

```text
data/raw_sensor_capture_nvidia.bag
```

2. 从 bag 中读取以下三个话题的 `header.stamp`：

- `/hikrobot_camera/rgb/compressed`
- `/livox/lidar_192_168_1_125`
- `/livox/lidar_192_168_1_148`

3. 对每一帧相机图像，寻找时间上最近的一帧雷达点云，并统计：

```text
camera_stamp - nearest_lidar_stamp
```

4. 为减少启动阶段和偶发异常帧的影响，对绝对值超过 `100 ms` 的样本进行剔除，保留稳态区间进行统计

### 9.3 bag 基本规模

本次用于离线分析的测试样本信息如下。

| 项目 | 数值 | 说明 |
| --- | --- | --- |
| bag 文件 | `data/raw_sensor_capture_nvidia.bag` | 本次在线录制原始数据包 |
| 录制时长 | `610 s` | `rosbag info` 结果 |
| bag 大小 | `4.8 GB` | 未压缩 |
| 相机话题 | `/hikrobot_camera/rgb/compressed` | `sensor_msgs/CompressedImage` |
| 雷达话题 1 | `/livox/lidar_192_168_1_125` | `livox_ros_driver2/CustomMsg` |
| 雷达话题 2 | `/livox/lidar_192_168_1_148` | `livox_ros_driver2/CustomMsg` |
| 相机样本数 | `10973` | bag 中图像消息数 |
| 雷达 125 样本数 | `5886` | bag 中点云消息数 |
| 雷达 148 样本数 | `5876` | bag 中点云消息数 |
| 相机在线频率 | 约 `20 Hz` | `rostopic hz` 现场测量 |
| 雷达 125 在线频率 | 约 `10.45 Hz` | `rostopic hz` 现场测量 |
| 雷达 148 在线频率 | 约 `10.46 Hz` | `rostopic hz` 现场测量 |

本次离线分析规则如下。

| 项目 | 规则 |
| --- | --- |
| 时间字段 | 使用各消息 `header.stamp` |
| 配对方式 | 每一帧相机图像匹配时间上最近的一帧雷达点云 |
| 统计量 | `camera_stamp - nearest_lidar_stamp` |
| 过滤规则 | 剔除绝对值超过 `100 ms` 的样本 |
| 过滤目的 | 去除启动阶段和偶发异常帧的影响，保留稳态统计 |

### 9.4 量化结果

量化结果汇总如下。

| 指标 | 相机 vs 雷达 `192.168.1.125` | 相机 vs 雷达 `192.168.1.148` |
| --- | --- | --- |
| 原始配对样本数 | `10973` | `10973` |
| 过滤后样本数 | `10838` | `10858` |
| 过滤保留比例 | `98.77%` | `98.95%` |
| 平均值 | `4.001 ms` | `3.954 ms` |
| 中位数 | `3.414 ms` | `4.114 ms` |
| 平均绝对值 | `26.588 ms` | `26.621 ms` |
| `p95` 绝对值 | `48.069 ms` | `48.092 ms` |
| 过滤后最大绝对值 | `98.658 ms` | `98.433 ms` |

为了避免对这些数值的含义产生误解，可将其进一步拆成“时基接近程度”和“采样相位差”两类解释。

| 观察项 | 数值特征 | 含义 |
| --- | --- | --- |
| 中位数 | 约 `3 到 4 ms` | 相机与雷达时间基准没有明显固定大偏差 |
| 平均值 | 约 `4 ms` | 从整体上看，系统时间与雷达 PTP 时基比较接近 |
| 平均绝对值 | 约 `26.6 ms` | 受 `20 Hz` 相机与 `10 Hz` 雷达不同采样频率影响明显 |
| `p95` 绝对值 | 约 `48 ms` | 接近雷达 `100 ms` 周期的一半，符合最近邻配对的采样相位差预期 |
| 最大绝对值 | 约 `98 ms` | 极端情况下接近过滤阈值，属于启动/抖动边缘样本 |

### 9.5 这些数值应该如何解释

这里最容易误解的是：

- `p95 ≈ 48 ms`

它并不直接等于“时钟偏差是 48 ms”。

原因在于：

- 相机频率约 `20 Hz`，周期约 `50 ms`
- 雷达频率约 `10 Hz`，周期约 `100 ms`
- 当用“每一帧相机去找最近一帧雷达”做最近邻配对时，天然会带来采样相位差

因此：

- 最近邻帧差的 `p95` 接近 `50 ms`，本身是符合采样频率关系的
- 这反映的是“相机帧与雷达帧不是同一频率、同一触发边沿采样”
- 不应简单解读为“时钟没有对齐”

真正更能反映“时基是否接近”的是中位数与平均值。

| 指标 | 当前观察 | 解释 |
| --- | --- | --- |
| 中位数 | 两台雷达相对相机都在 `3 到 4 ms` 左右 | 没有出现明显固定大偏差 |
| 平均值 | 两台雷达相对相机都在 `4 ms` 左右 | 相机系统时间与雷达 PTP 时基整体比较接近 |

这说明：

- 相机时间和雷达时间没有明显的固定大偏差
- 当前系统中，相机虽然没有直接参加 PTP，但其系统时间与雷达 PTP 时基处于比较接近的状态

### 9.6 这一部分的工程结论

因此，对相机应作如下表述：

- 相机没有直接使用 PTP
- 相机仍然是系统时间打戳
- 但当前系统时间与雷达 PTP 时基没有表现出明显的大偏移
- 所以工程上可以认为相机与雷达已经处于统一时基附近

但同时也必须明确：

- 这不是严格硬件同步
- 相机与雷达的逐帧最近邻时间差仍然会明显受到 `20 Hz` 对 `10 Hz` 采样相位差的影响
- 若后续任务需要更严格的时空对齐，仍应进一步估计相机固定延迟或采用支持硬同步的相机

---

## 10. 当前结论

### 10.1 已确认正确的部分

当前可以确认：

1. `eth0` 支持硬件时间戳，且存在 `/dev/ptp0`
2. `collect_data_with_ptp.launch` 已正确拉起 `ptp4l/phc2sys`
3. 当前 PTP 域中，主机侧时钟在充当 grandmaster
4. 两台 Mid-360 都在线，并稳定发布点云与 IMU 数据
5. 两台 Mid-360 的原始 Livox 数据包 `time_type = 1`
6. 因此两台雷达当前都在使用 PTP/gPTP 时间同步
7. 相机虽然没有直接参加 PTP，但从离线 bag 统计看，其时间与雷达时间基准没有明显固定大偏差

### 10.2 当前仍未量化的部分

本次验证已经证明“PTP 已启用且生效”，但还没有量化以下指标：

- 两台雷达之间的实际同步误差分布
- 同步抖动是否达到工程目标
- 相机曝光时刻相对于驱动打戳时刻的固定延迟
- 点云与相机之间的真实物理采样时刻误差

也就是说：

- 当前结论可以确认“已切入 PTP 时基”
- 也可以确认“相机与雷达处于统一时基附近”
- 但不能仅凭这一步推断“相机与雷达已经实现严格硬同步”或“同步精度已经达到某个具体微秒级指标”

---

## 11. 后续建议

若需要继续深入验证，可按以下顺序进行：

1. 统计双雷达点云 `timebase/header.stamp` 的帧间差值与抖动
2. 统计双雷达 IMU 的时间差分布
3. 若需要评估相机真实时延，进一步统计 `bag record time - header.stamp` 分布并结合视觉事件做外参级对齐验证
4. 若后续接入外部 grandmaster，再将 `ptp_clock_sync_direction` 显式切换为 `phc-to-system` 重新验证

---

## 12. 一句话结论

当前这套双 Mid-360 在线系统中，主机侧 PTP 链路已经建立，两台雷达都在参与 PTP 报文交互，且其 Livox 原始数据包中的 `time_type` 均为 `1`，因此可以确认两台雷达目前确实在使用 PTP/gPTP 时间同步；相机虽然不直接使用 PTP，但从离线 bag 统计看，其时间与雷达 PTP 时基已经处于统一基准附近，只是尚不属于严格硬件同步。