# PTP / 时间同步对话总结

日期：2026-03-11

## 1. 目标

本次对话的目标是确认以下问题：

- 当前网络中是否存在外部 PTP master 时钟
- 主机网卡是否支持硬件时间戳
- 当前 `ptp4l` 的角色是什么
- 雷达、内置 IMU、USB 相机在 ROS 中分别使用什么时间源
- 为什么主机虽然是 PTP master，仍然可能需要将系统时间同步到 PTP

---

## 2. 已确认事实

### 2.1 网卡支持硬件时间戳

通过以下命令确认：

```bash
ethtool -T eth0
```

得到的关键信息：

- 存在 `hardware-transmit`
- 存在 `hardware-receive`
- 存在 `hardware-raw-clock`
- `PTP Hardware Clock: 0`

这说明：

- `eth0` 支持硬件时间戳
- 主机存在 PTP 硬件时钟（PHC）
- 该 PHC 通常对应 `/dev/ptp0`

此前也已确认系统中存在：

```bash
/dev/ptp0
```

---

### 2.2 当前没有检测到外部 PTP master

运行 `ptp4l` 后，用户贴出的日志为：

```text
sudo ptp4l -i eth0 -m
ptp4l[7713.203]: selected /dev/ptp0 as PTP clock
ptp4l[7713.204]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[7713.204]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[7713.204]: port 1: link down
ptp4l[7713.204]: port 1: LISTENING to FAULTY on FAULT_DETECTED (FT_UNSPECIFIED)
ptp4l[7713.204]: selected local clock 001114.fffe.173441 as best master
ptp4l[7713.204]: assuming the grand master role
```

这说明：

- `ptp4l` 选择了本机的 `/dev/ptp0` 作为 PTP 时钟
- 当前没有收到比本机更优的外部 master 信息
- 本机正在承担 **PTP grandmaster** 角色

结论：

- 当前网络上**没有证据表明存在外部 PTP master**
- 现在是本机在充当 master

---

## 3. 传感器时间源分析

用户已明确说明：

- 雷达驱动发布的是**设备时间**
- 内置 IMU 与雷达**同源**
- USB 相机使用 `ros::Time::now()` 获取时间

因此当前系统中的时间源可分为两类。

### 3.1 雷达 + 内置 IMU

- 使用的是雷达设备时间
- 若雷达已配置为 PTP slave，则其设备时间会跟随 PTP 时间
- 因为 IMU 与雷达同源，所以雷达与 IMU 的时间通常天然一致

### 3.2 USB 相机

- 使用的是 `ros::Time::now()`
- 在 ROS1 中，这通常对应 Linux 主机系统时间，即 `CLOCK_REALTIME`
- 它不是直接使用 PTP 设备时间

---

## 4. 关键区别：PTP 时钟 != 主机系统时钟

这是本次对话中最核心的点。

即使当前主机是 PTP master，也只是表示：

- 本机的 **PTP 硬件时钟**（PHC，例如 `/dev/ptp0`）在 PTP 域中担任主时钟

但这**不等于**：

- Linux 主机系统时间 `CLOCK_REALTIME` 已自动与该 PTP 时钟一致

也就是说，主机中实际上可能同时存在两套钟：

1. **PTP 硬件时钟（PHC）**：`/dev/ptp0`
2. **系统时钟**：`CLOCK_REALTIME`

其中：

- 雷达/IMU 看的是设备/PTP 时间
- USB 相机驱动的 `ros::Time::now()` 看的是系统时间

如果不做额外同步，这两套时间**不一定完全一致**。

---

## 5. 为什么还需要 `phc2sys`

`phc2sys` 的作用不是再去“找 master”，而是：

- 把 Linux 系统时钟同步到 PHC / PTP 时间

也就是将：

```text
CLOCK_REALTIME <- /dev/ptp0
```

常见命令为：

```bash
sudo ptp4l -i eth0 -m
sudo phc2sys -s /dev/ptp0 -c CLOCK_REALTIME -m
```

作用解释：

- `ptp4l`：让 `/dev/ptp0` 参与并维持 PTP 时钟关系
- `phc2sys`：把主机系统时间拉到 `/dev/ptp0` 这套 PTP 时间基准上

这样一来：

- 雷达/IMU：继续用设备/PTP 时间
- 相机：继续用 `ros::Time::now()`
- 但系统时间已经被校准到 PTP

最终三者会尽量落在同一个时间基准上。

---

## 6. 不同配置下的实际效果

### 情况 A：不运行 `phc2sys`

- 雷达/IMU：PTP / 设备时间
- 相机：主机系统时间

结果：

- 雷达与 IMU 彼此同步
- 相机与它们**不是严格同一时基**
- 可能存在固定偏差或慢漂

### 情况 B：运行 `phc2sys`

- 雷达/IMU：设备/PTP 时间
- 相机：系统时间，但系统时间已经同步到 PTP

结果：

- 全系统接近统一时基
- 适合 ROS 中做时间对齐
- 但 USB 相机仍然不是严格的硬件同步

---

## 7. 关于 USB 相机的限制

即使运行了 `phc2sys`，USB 相机通常仍然只能做到“统一时基”，而不是严格硬同步。

原因包括：

- USB 传输延迟
- 驱动排队延迟
- 图像到达主机后才打戳
- `ros::Time::now()` 的打戳时刻不一定等于曝光中点

所以：

- 雷达 + 内置 IMU：通常同步质量较高
- USB 相机：通常只能做到较好的软件时间对齐

---

## 8. ROS 在这里扮演的角色

ROS / `roslaunch` 本身**不会自动统一各个设备的时钟**。

ROS 只是把驱动填入消息头的 `header.stamp` 发出去。

也就是说：

- 如果驱动用设备时间，ROS 就发设备时间
- 如果驱动用 `ros::Time::now()`，ROS 就发系统时间

因此跨传感器时间统一的关键不是 `roslaunch`，而是：

- 驱动如何打时间戳
- 系统时钟是否已同步到 PTP

---

## 9. 本次对话中创建的辅助文件

为了便于检查 PTP 状态，工作区中新增了以下文件：

- `scripts/check_ptp.sh`
- `scripts/README_CHECK_PTP.md`

用途：

- 检查网卡时间戳能力
- 检查 `/dev/ptp*`
- 可选抓取 PTP 报文
- 可选运行 `pmc`
- 可选运行 `ptp4l`

---

## 10. 最终结论

### 已确认结论

1. 当前主机网卡 `eth0` 支持硬件时间戳
2. 当前主机存在 PTP 硬件时钟 `/dev/ptp0`
3. 当前 `ptp4l` 运行结果表明：**本机正在充当 PTP grandmaster**
4. 雷达和内置 IMU 使用同源设备时间
5. USB 相机使用 `ros::Time::now()`，即主机系统时间

### 关键结论

虽然当前主机是 PTP master，但：

- PTP master 指的是 `/dev/ptp0` 这类 PHC
- 并不自动意味着 `CLOCK_REALTIME` 已经和 PTP 对齐

因此，如果希望：

- 雷达 / IMU 的设备时间
- 与 USB 相机的 `ros::Time::now()`

尽量处于同一时间基准下，就应该运行：

```bash
sudo ptp4l -i eth0 -m
sudo phc2sys -s /dev/ptp0 -c CLOCK_REALTIME -m
```

---

## 11. 推荐操作

推荐在启动 ROS 传感器节点前，先完成以下步骤：

```bash
sudo ptp4l -i eth0 -m
sudo phc2sys -s /dev/ptp0 -c CLOCK_REALTIME -m
```

然后再启动：

- 雷达驱动
- IMU 驱动
- 相机驱动
- 后续融合节点

这样可以让 USB 相机与雷达/IMU 尽量处于统一时基下。

---

## 12. 一句话总结

**你现在已经有 PTP 硬件时钟，并且本机是 PTP master；但若不把系统时钟同步到这套 PTP 时间，USB 相机的 `ros::Time::now()` 仍然可能和雷达/IMU 的设备时间不在同一时基上。**
