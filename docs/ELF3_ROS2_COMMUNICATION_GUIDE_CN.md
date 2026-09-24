# ELF3 机器人 ROS 2 通信与运行架构入门

本文面向第一次接触 ROS 2、需要查看和排查 ELF3 机器人通信的开发者。
内容以 `elf3-19` 在 2026-09-03 的实际运行环境为基础，并结合当前
`bxi_rl_controller_ros2_example` 源码整理。

> 安全边界：本文的 `list`、`info`、`echo`、`hz`、`interface show`、`ps`、
> `systemctl show/status` 和日志查看命令都是只读检查。不要在不了解消息含义时执行
> `ros2 topic pub`、`ros2 service call`、硬件 launch、参数修改或进程启停；这些操作
> 可能向真实机器人发送指令。

## 1. 先建立整体认识

ELF3 上的 ROS 2 可以理解成一个机器人内部的消息网络：

```text
手柄、传感器、电机
        │
        ▼
节点 Node：执行具体工作的程序
        │
        ▼
话题 Topic / 服务 Service：节点之间的数据接口
        │
        ▼
RMW：ROS 2 对通信中间件的统一接口
        │
        ▼
DDS（当前为 CycloneDDS）：节点发现和消息传输
        │
        ▼
Linux 网络接口、UDP、共享内存
```

当前最重要的控制闭环是：

```text
实体手柄或 CRSF 遥控器
        │
        ▼
/remote_controller
        │ 发布 /motion_commands
        ▼
/bxi_example_py_elf3_demo
        │ 发布 /hardware/actuators_cmds
        ▼
/hardware_elf3
        │
        └──────────────► 真实电机

真实电机和 IMU
        │
        ▼
/hardware_elf3
        │ 发布 /hardware/actuator_states、/hardware/imu_data
        ▼
/bxi_example_py_elf3_demo
        │
        └── 根据状态和手柄输入计算下一帧控制指令
```

这是闭环控制：控制器不能只发送命令，还必须持续读取机器人状态，再计算下一帧命令。

## 2. ROS 2 的基本名词

### 2.1 Package：软件包

Package 是磁盘上的代码、配置、消息定义和启动文件集合。当前相关软件包包括：

- `remote_controller`：读取手柄、CRSF 或键盘输入；
- `bxi_example_py_elf3`：ELF3 策略、状态机和控制器；
- `bxi_example_bms`：电池状态示例；
- `bxi_depth_camera`：深度相机管理；
- `hardware_elf3`：ELF3 硬件驱动，来自二进制 BXI ROS 2 包；
- `communication`：BXI 自定义 ROS 2 消息和服务类型。

Package 本身只是安装在磁盘上的软件，不代表程序正在运行。

### 2.2 Workspace：工作区

机器人上的 BXI 工作区是：

```text
/home/bxi/bxi_ws/bxi_rl_controller_ros2_example
```

典型 ROS 2 工作区包含：

```text
src/      源码
build/    编译过程文件
install/  编译安装结果和 setup.bash
log/      colcon 编译、测试日志
```

本机开发仓库路径则是：

```text
/home/user-kevien/holomotion/bxi_rl_controller
```

两者用途和版本可能不同。排查真机时，应先确认机器人正在使用哪个 `install/`。

### 2.3 Node：节点

Node 是正在运行的程序实例。使用下面的命令查看当前 ROS 网络中的节点：

```bash
ros2 node list
```

`elf3-19` 的完整控制程序运行时，已确认可见这些核心节点：

| 节点 | 职责 |
|---|---|
| `/remote_controller` | 读取手柄或 CRSF，发布运动命令，并处理系统启动/停止事件 |
| `/bxi_example_py_elf3_demo` | 主控制器、策略和动作状态机 |
| `/hardware_elf3` | 连接真实电机和 IMU，发布硬件反馈并接收控制命令 |
| `/bxi_bms` | 电池管理系统底层接口 |
| `/bxi_example_bms` | 订阅并显示电池电压、电流、电量和温度 |
| `/depth_camera_manager` | 发现、启动和管理深度相机 |

查看一个节点的发布者、订阅者和服务：

```bash
ros2 node info /hardware_elf3
ros2 node info /bxi_example_py_elf3_demo
ros2 node info /remote_controller
```

### 2.4 Topic：话题

Topic 是持续传输消息的数据频道。一个节点作为 Publisher 发布数据，另一个节点作为
Subscriber 订阅数据。

```text
Publisher ──消息──► /topic_name ──消息──► Subscriber
```

例如：

```text
/hardware_elf3
    ──ActuatorStates──► /hardware/actuator_states
                              ──► /bxi_example_py_elf3_demo
```

常用命令：

```bash
# 列出话题及消息类型
ros2 topic list -t

# 查看发布者和订阅者数量
ros2 topic info /hardware/actuator_states -v

# 读取一帧数据
ros2 topic echo --once /hardware/actuator_states

# 测量消息频率
ros2 topic hz /hardware/actuator_states

# 查看消息字段定义
ros2 interface show communication/msg/ActuatorStates
```

重要：`ros2 topic list` 中出现一个名字，不代表它一定正在收到数据。只有订阅者而没有
发布者时，这个话题也可能出现在列表中。应继续检查：

```bash
ros2 topic info /hardware/odom -v
```

如果 `Publisher count: 0`，表示当前没有节点发布它，`echo` 会一直等待。

### 2.5 Message：消息

Message 是话题中每一帧数据的结构。常见标准消息包括：

- `sensor_msgs/msg/Imu`：IMU 姿态、角速度和加速度；
- `sensor_msgs/msg/JointState`：关节名称、位置、速度和力矩；
- `geometry_msgs/msg/Twist`：线速度和角速度；
- `nav_msgs/msg/Odometry`：里程计。

BXI 自定义消息包括：

- `communication/msg/MotionCommands`：手柄运动和按键命令；
- `communication/msg/ActuatorStates`：执行器状态；
- `communication/msg/ActuatorCmds`：执行器控制命令；
- `communication/msg/BatteryStates`：电池状态。

话题名相同但消息类型不同，发布者和订阅者也不能正常通信。

### 2.6 Service：服务

Service 适合一次请求、一次响应的操作，而不是连续数据流：

```text
Client ──Request──► Service Server
Client ◄─Response── Service Server
```

当前硬件节点提供：

```text
/hardware/robot_reset [communication/srv/RobotReset]
```

控制器是这个服务的 Client，硬件节点是 Server。服务调用可能改变机器人状态，因此本文
只建议查看，不提供真机调用命令：

```bash
ros2 service list -t
ros2 service type /hardware/robot_reset
```

### 2.7 Parameter、Action 和 Launch

- Parameter：节点的运行配置，例如相机分辨率和硬件开关；
- Action：可反馈进度、可取消的长任务，常用于导航或长动作；
- Launch：一次启动一组节点的启动描述。

例如真机 launch `example_demo_hw.launch.py` 会组织启动：

```text
深度相机管理节点 + hardware_elf3 + bxi_example_py_elf3_demo
```

带 `hw` 的 launch 会涉及真实硬件，不能把它当成普通查看命令。

## 3. 当前 ELF3 的节点和话题关系

### 3.1 手柄输入链路

```text
手柄/CRSF
   │
   ▼
/remote_controller
   │ Publisher
   ▼
/motion_commands [communication/msg/MotionCommands]
   │ Subscriber
   ▼
/bxi_example_py_elf3_demo
```

只读检查：

```bash
ros2 topic info /motion_commands -v
ros2 topic echo --once /motion_commands
```

### 3.2 控制命令链路

```text
/bxi_example_py_elf3_demo
   │ Publisher
   ▼
/hardware/actuators_cmds [communication/msg/ActuatorCmds]
   │ Subscriber
   ▼
/hardware_elf3
   │
   ▼
真实电机
```

`actuators_cmds` 是会送往硬件的控制输出。可以用 `info` 检查连接关系；不要手动向它
发布消息：

```bash
ros2 topic info /hardware/actuators_cmds -v
```

### 3.3 硬件反馈链路

`/hardware_elf3` 当前实际发布：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/hardware/actuator_states` | `communication/msg/ActuatorStates` | 控制器使用的电机状态 |
| `/hardware/joint_states` | `sensor_msgs/msg/JointState` | 通用关节状态和状态监控 |
| `/hardware/imu_data` | `sensor_msgs/msg/Imu` | 机器人姿态和惯性数据 |
| `/canfd_packet/rx` | `communication/msg/CANFDPacket` | CAN-FD 接收数据 |
| `/rosout` | `rcl_interfaces/msg/Log` | ROS 日志 |

`/bxi_example_py_elf3_demo` 订阅：

```text
/hardware/actuator_states
/hardware/imu_data
/hardware/odom
/hardware/touch_sensor
/motion_commands
/cmd_vel
/hardware/actuators_cmds_override
```

其中 `/hardware/odom`、`/hardware/touch_sensor`、`/cmd_vel` 和
`/hardware/actuators_cmds_override` 是可选输入。话题名可能存在，但当前是否真的有
Publisher，必须用 `ros2 topic info -v` 判断。

### 3.4 状态机和电池

控制器发布：

```text
/hardware/state_machine_info [std_msgs/msg/String]
```

查看当前状态机信息：

```bash
ros2 topic echo --once /hardware/state_machine_info
```

电池链路为：

```text
/bxi_bms
   ──► /battery_states [communication/msg/BatteryStates]
            ──► /bxi_example_bms
```

## 4. DDS、RMW 和 Domain

### 4.1 DDS 做什么

DDS（Data Distribution Service）是 ROS 2 底层使用的分布式通信标准，负责：

- 自动发现节点；
- 建立 Publisher 和 Subscriber 连接；
- 在进程或机器之间传输消息；
- 管理可靠性、缓存深度和其他 QoS 策略；
- 选择网卡、组播和共享内存等传输方式。

ROS 1 通常依赖中心化的 `roscore`。ROS 2 的 DDS 使用分布式发现，通常不需要一个中心
Master。

### 4.2 RMW 做什么

RMW（ROS Middleware Interface）是 ROS 2 到具体 DDS 实现的统一接口：

```text
rclcpp / rclpy
      │
      ▼
     RMW
      │
      ├── CycloneDDS
      ├── Fast DDS
      └── 其他 DDS 实现
```

当前 `elf3-19` 使用 CycloneDDS：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 4.3 Domain 做什么

`ROS_DOMAIN_ID` 是 DDS 的逻辑隔离编号，可以理解为通信房间号：

```text
Domain 0  ── 一套互相可见的节点和话题
Domain 49 ── 另一套互相可见的节点和话题
```

不同 Domain 中，即使节点名和话题名相同，也默认互相看不见。`elf3-19` 当前控制服务
使用 `ROS_DOMAIN_ID=49`。

Domain 不是 IP 地址，也不会出现在话题名称中。它只影响 DDS 发现和通信隔离。

### 4.4 当前为什么绑定 `lo`

当前 systemd 服务还设置了：

```xml
<NetworkInterface name="lo" multicast="true"/>
```

`lo` 是 Linux 本机回环接口。因此 Domain 49 的这套控制通信只在机器人本机发现，不会
直接通过以太网或 Wi-Fi 对外广播。正确查看方式是先 SSH 到机器人，再在机器人本机运行
ROS 2 CLI。

虽然同时设置了 `ROS_LOCALHOST_ONLY=0`，但 CycloneDDS 的显式 `lo` 配置仍决定实际使用
回环接口。

## 5. 在 elf3-19 上建立正确的只读检查环境

SSH 登录机器人后，在同一个终端依次执行：

```bash
source /opt/ros/humble/setup.bash
source /home/bxi/bxi_ws/bxi_rl_controller_ros2_example/install/setup.bash

export ROS_DOMAIN_ID=49
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'

ros2 daemon stop
ros2 node list
```

各行作用如下：

| 命令 | 作用 |
|---|---|
| `source /opt/ros/humble/setup.bash` | 加载 ROS 2 Humble 的命令、库和标准消息 |
| `source .../install/setup.bash` | 叠加 BXI 编译后的软件包、自定义消息和 launch |
| `ROS_DOMAIN_ID=49` | 进入控制服务所在的 DDS Domain |
| `ROS_LOCALHOST_ONLY=0` | 不使用 ROS 变量强制本机限制 |
| `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | 选择与服务相同的 CycloneDDS 实现 |
| `CYCLONEDDS_URI=...lo...` | 选择与服务相同的本机回环 DDS 接口 |
| `ros2 daemon stop` | 清理 CLI 后台发现进程，下一条命令按当前环境重新启动 |
| `ros2 node list` | 读取当前 ROS Graph 中的节点 |

`source` 和 `export` 只改变当前 Shell，不会启动机器人控制程序。关闭终端后这些设置不会
保留，新终端需要重新执行。

`ros2 daemon stop` 只停止 ROS 2 CLI 使用的后台图发现进程，不会停止
`hardware_elf3`、控制器或电机。

机器人当前实际 systemd 环境可以只读查询，避免以后 Domain 改动后继续硬编码 49：

```bash
systemctl show ros_elf_launch.service -p Environment --no-pager
```

注意：仓库中的 service 模板可能仍是 `ROS_DOMAIN_ID=XX` 占位符。真机上已安装的
`/etc/systemd/system/ros_elf_launch.service` 才是当前运行配置的依据。

## 6. 机器人从开机到控制的运行流程

### 6.1 开机常驻阶段

systemd 服务：

```text
ros_elf_launch.service
```

它首先启动 `/remote_controller`。此时手柄读取程序存在，但完整硬件控制程序不一定已经
启动。因此可能看到 `/remote_controller`，却看不到 `/hardware_elf3` 和
`/hardware/actuator_states`。

只读检查服务：

```bash
systemctl status ros_elf_launch.service --no-pager -l
```

### 6.2 手柄启动阶段

配置中的系统启动事件会启动：

```text
example_demo_hw.launch.py
    ├── depth_camera_manager
    ├── hardware_elf3
    └── bxi_example_py_elf3_demo

bms.launch.py
    ├── bxi_bms
    └── bxi_example_bms
```

这些是硬件控制进程。必须由现场操作者按照安全流程启动，不能为了“让话题出现”而远程
盲目启动。

### 6.3 运行阶段

完整控制程序运行后，预期看到：

```bash
ros2 node list
```

```text
/bxi_bms
/bxi_example_bms
/bxi_example_py_elf3_demo
/depth_camera_manager
/hardware_elf3
/remote_controller
```

并且：

```bash
ros2 topic info /hardware/actuator_states -v
```

应至少有：

```text
Publisher count: 1
Subscription count: 1
```

### 6.4 停止阶段

停止事件会向硬件、控制器、相机、BMS 等进程发送退出信号。`hardware_elf3` 退出后：

- `/hardware_elf3` 从节点列表消失；
- `/hardware/actuator_states` 的 Publisher 消失；
- 即使某个 Subscriber 仍声明该话题，也不会再有硬件数据。

因此“话题不见了”或“Publisher count 为 0”首先要区分：

1. 控制程序根本没有启动或已经停止；
2. 当前终端的 Domain/RMW/DDS 接口不匹配；
3. 节点在运行，但硬件驱动发生异常。

## 7. 日志在哪里

### 7.1 真机控制日志

手柄启动的控制程序日志默认重定向到：

```text
/var/log/bxi_log/YYYY-MM-DD_HH-MM-SS_elf.log
```

查看最新控制日志：

```bash
latest_log=$(find /var/log/bxi_log -maxdepth 1 -type f -name '*_elf.log' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)
less "$latest_log"
```

持续查看：

```bash
tail -f "$latest_log"
```

### 7.2 BMS 日志

```text
/var/log/bxi_log/bms_YYYY-MM-DD_HH-MM-SS_bms.log
```

### 7.3 手柄 systemd 日志

```bash
journalctl -u ros_elf_launch.service --no-pager -n 200
journalctl -u ros_elf_launch.service -f
```

### 7.4 ROS 2 和构建日志的区别

- `~/.ros/log/` 或 root 的 `/root/.ros/log/`：ROS 2 运行时日志；
- 工作区 `log/`：`colcon build/test` 的构建和测试日志；
- `/var/log/bxi_log/`：当前真机启动流程主动重定向的控制和 BMS 日志；
- rosbag：话题数据录制，不是文本日志。

## 8. rosbag 是什么以及如何手动录制

rosbag 会按时间保存 ROS 2 话题，可以用于复现和离线分析。当前 BXI 启动流程默认不会
自动录包。

录制前必须先建立与控制服务一致的 Domain/RMW/DDS 环境。建议只录必要话题，避免用
`-a` 把高带宽相机图像一起录入：

```bash
mkdir -p /home/bxi/rosbags
cd /home/bxi/rosbags
ros2 bag record \
  /motion_commands \
  /hardware/actuator_states \
  /hardware/actuators_cmds \
  /hardware/imu_data \
  /hardware/state_machine_info
```

按 `Ctrl+C` 停止并完成写盘。默认生成：

```text
/home/bxi/rosbags/rosbag2_YYYY_MM_DD-HH_MM_SS/
```

查看录包信息：

```bash
ros2 bag info /home/bxi/rosbags/rosbag2_YYYY_MM_DD-HH_MM_SS
```

播放 rosbag 会重新发布其中的数据。控制话题的 rosbag 不能连接真实机器人随意播放；
应先在隔离的离线分析或仿真环境中确认用途。

## 9. 常见故障的只读排查顺序

### 情况 A：`ros2 node list` 是空的

先检查环境：

```bash
echo "$ROS_DOMAIN_ID"
echo "$ROS_LOCALHOST_ONLY"
echo "$RMW_IMPLEMENTATION"
echo "$CYCLONEDDS_URI"
systemctl show ros_elf_launch.service -p Environment --no-pager
```

确保终端和服务一致，然后：

```bash
ros2 daemon stop
ros2 node list
```

### 情况 B：能看到 `/remote_controller`，看不到 `/hardware_elf3`

检查进程和最近日志：

```bash
ps -eo pid,lstart,args | grep -E 'hardware_elf3|bxi_example_py_elf3_demo' | grep -v grep
ls -lt /var/log/bxi_log/*_elf.log | head
```

这通常表示完整控制程序尚未启动或已经停止。不要仅为显示话题而自行启动真机控制程序。

### 情况 C：话题名存在，但 `echo` 没有输出

```bash
ros2 topic info /hardware/actuator_states -v
```

重点检查：

- `Publisher count` 是否大于 0；
- Publisher 是否是 `/hardware_elf3`；
- 消息类型是否为 `communication/msg/ActuatorStates`；
- `hardware_elf3` 是否持续运行；
- 日志中是否存在 CAN、电机超时、IMU 或保护错误。

### 情况 D：看到相机和感知节点，却看不到控制节点

不要据此判断控制节点不存在。它们可能位于不同 Domain 或使用不同 DDS 接口。先查询
systemd 的实际环境，再匹配 Domain、RMW 和 `CYCLONEDDS_URI`。

### 情况 E：判断话题是否真的在工作

按以下顺序：

```bash
ros2 topic list -t
ros2 topic info /hardware/actuator_states -v
ros2 topic hz /hardware/actuator_states
ros2 topic echo --once /hardware/actuator_states
```

含义分别是：名字和类型、连接关系、实时频率、实际内容。

## 10. 常用只读命令速查

```bash
# ROS Graph
ros2 node list
ros2 topic list -t
ros2 service list -t

# 节点详情
ros2 node info /hardware_elf3
ros2 node info /bxi_example_py_elf3_demo
ros2 node info /remote_controller

# 硬件反馈
ros2 topic info /hardware/actuator_states -v
ros2 topic echo --once /hardware/actuator_states
ros2 topic hz /hardware/actuator_states

# 手柄输入
ros2 topic info /motion_commands -v
ros2 topic echo --once /motion_commands

# 消息定义
ros2 interface show communication/msg/ActuatorStates
ros2 interface show communication/msg/ActuatorCmds
ros2 interface show communication/msg/MotionCommands

# 服务和进程环境
systemctl status ros_elf_launch.service --no-pager -l
systemctl show ros_elf_launch.service -p Environment --no-pager

# 日志
journalctl -u ros_elf_launch.service --no-pager -n 200
ls -lt /var/log/bxi_log | head
```

## 11. 阅读结果时记住四个判断层级

```text
1. 进程层：Linux 进程是否存在？
2. 节点层：当前 ROS Domain 中能否发现 Node？
3. 连接层：Topic 是否有正确的 Publisher 和 Subscriber？
4. 数据层：消息是否持续发布，字段值是否合理？
```

不要只看其中一层。例如：

- 进程存在但 `node list` 看不到：优先检查 Domain、RMW 和 DDS 接口；
- 话题存在但 Publisher 为 0：只是订阅者声明了名称，没有数据源；
- Publisher 和 Subscriber 都存在但没有数据：检查频率、QoS 和节点日志；
- 数据持续存在但机器人行为异常：再进入消息字段、控制状态机和硬件保护层排查。
