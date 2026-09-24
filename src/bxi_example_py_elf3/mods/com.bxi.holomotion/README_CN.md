# ELF3 HoloMotion + Pico + BXI MuJoCo 完整启动指南

本文档是当前本机 ELF3 HoloMotion Pico 仿真的唯一推荐启动流程。它对应已经实际
运行过的以下链路：

```text
Pico / XRoboToolkit body_poses[24, 7]
  -> 工作站 HoloRetarget
  -> 50 Hz reference_qpos[36]，ZMQ tcp://127.0.0.1:6001
  -> HoloMotion 原生 BXI Mod
  -> 43k ONNX policy + KV cache
  -> BXI MotorFrame
  -> MuJoCo（SONIC ELF3 XML）
```

这套流程只运行本地仿真，不连接 CAN，也不会向真机发送动作。

## 0. 本机日常快速启动

这里的“一键启动”指：MuJoCo 控制器和键盘节点已经运行后，按一次 `Shift+3`，Mod
自动启动本机 CPU HoloRetarget、6001 reference 发布端并按需加载 43k policy。Pico
App 仍需由操作者提前打开并开启 `Full body` 和 `Send`；MuJoCo 窗口与需要前台焦点的
键盘节点也不会由 `Shift+3` 自动打开。

日常启动只需要两个终端，不再需要手工启动 HoloRetarget：

终端 A：

```bash
cd /home/user-kevien/holomotion
source bxi_rl_controller/setup_holomotion_bxi.bash
ros2 launch bxi_example_py_elf3 example_launch_demo.py \
  model_file:=/home/user-kevien/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/data/mujoco_simulation/elf3.xml
```

终端 B：

```bash
cd /home/user-kevien/holomotion
source bxi_rl_controller/setup_holomotion_bxi.bash
ros2 launch remote_controller remote_controller_keyboard.launch.py
```

看到 MuJoCo 中机器人处于 Normal 后，点击终端 B 取得键盘焦点，再按 `Shift+3`。日志
从 `waiting_reference` 进入 `tracking` 后，Pico 动作才会送入策略。完整的首次安装、
检查、退出和故障处理见后续章节。

## 1. 当前固定配置

| 项目 | 当前值 |
| --- | --- |
| HoloMotion 仓库 | `/home/user-kevien/holomotion` |
| 独立 BXI 工作区 | `/home/user-kevien/holomotion/bxi_rl_controller` |
| ROS Domain | `74` |
| MuJoCo XML | `/home/user-kevien/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/data/mujoco_simulation/elf3.xml` |
| HoloMotion 模型 | `assets/model_43000.onnx`，43k 权重 |
| Pico 发布端 Python | `/home/user-kevien/miniforge3/envs/holomotion_teleop/bin/python` |
| Reference 端口 | `tcp://127.0.0.1:6001` |
| Reference 频率 | `50 Hz` |
| Pico 源数据超时 | `0.6 s` |
| 连续参考启动门 | 当前帧 + 10 帧未来参考，共 11 帧；固定未来等待约 `200 ms` |
| 首帧 IK 预收敛 | 同一 Pico 首帧执行 `24` 次，仅新序列执行 |
| Tracking 切换 | 首个有效模型输出后，从 Normal 线性混合 `0.4 s` |
| HoloMotion 键位 | `Shift+3` |
| 长时间运行 | 第 8128 个策略步重置 KV cache 和步号，保持 `tracking` |

这里使用的是 SONIC 仿真的 ELF3 XML，但只借用该 XML 文件；不要 source
`/home/user-kevien/bxi_rl_controller_ros2_example/install/setup.bash`。HoloMotion 和
GVHMR Web 仍然使用两套隔离的控制器工作区。

43k 模型由 Mod 自动加载，不需要在启动命令中再传一次模型路径。Mod 中的模型文件：

```text
/home/user-kevien/holomotion/bxi_rl_controller/src/
  bxi_example_py_elf3/mods/com.bxi.holomotion/assets/model_43000.onnx
```

它与仓库中的 `checkpoints/Holomotion-elf3/model_43000.onnx` 是同一 inode 的硬链接，
不会额外占用约 1.64 GB 磁盘空间。

## 2. 首次启动前检查

先打开一个全新的系统终端，不要使用已经 source 过 GVHMR Web 工作区的终端。

```bash
cd /home/user-kevien/holomotion

test -f bxi_rl_controller/install/setup.bash
test -f bxi_rl_controller/src/bxi_example_py_elf3/mods/com.bxi.holomotion/assets/model_43000.onnx
test -f /home/user-kevien/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/data/mujoco_simulation/elf3.xml

/home/user-kevien/miniforge3/envs/holomotion_teleop/bin/python -c \
  "import holoretarget, xrobotoolkit_sdk, zmq; print('Pico 环境正常')"
```

以上命令都应成功。检查模型是否仍为当前 43k 权重：

```bash
stat -c '%n  %s bytes  inode=%i' \
  /home/user-kevien/holomotion/checkpoints/Holomotion-elf3/model_43000.onnx \
  /home/user-kevien/holomotion/bxi_rl_controller/src/bxi_example_py_elf3/mods/com.bxi.holomotion/assets/model_43000.onnx
```

两个文件当前应都是 `1640343544 bytes`，inode 应相同。

如果修改过 BXI Mod 或键位配置，先重新构建：

```bash
cd /home/user-kevien/holomotion/bxi_rl_controller
source setup_holomotion_bxi.bash
colcon build --merge-install --symlink-install \
  --packages-select bxi_example_py_elf3 remote_controller
```

普通启动不需要每次重新构建。

首次部署时还要安装本地 CPU HoloRetarget 服务和仅限该服务的免密规则：

```bash
sudo install -o root -g root -m 0644 \
  deployment/holomotion_teleop/holomotion-retarget-local.service \
  /etc/systemd/system/holomotion-retarget.service
sudo install -o root -g root -m 0440 \
  deployment/holomotion_teleop/holomotion-retarget-local.sudoers \
  /etc/sudoers.d/holomotion-retarget-local
sudo systemctl daemon-reload
```

这三条命令只在首次安装或服务文件变化时需要管理员密码。日常按 `Shift+3` 不会再
询问密码。sudoers 只放行该 unit 的 `start` 和 `stop` 两条完整命令；Mod 使用
`sudo -n`，配置缺失时会立即失败并保持在 Normal，不会打开交互式密码提示。

## 3. Pico 端准备

Pico 和电脑应连接同一低延迟局域网。在电脑上查看当前 IPv4 地址：

```bash
ip -br -4 addr show up
```

在 Pico 的 XRoboToolkit App 中：

1. 将 `PC Service` 设为电脑当前局域网 IP；
2. 确认状态变为 `WORKING`；
3. 开启 `Head`、`Controller`、`Full body` 和 `Send`；
4. 穿戴并校准两个脚部 Tracker；
5. 站稳，确认 App 中能看到完整身体骨架。

Pico 可以在启动工作站程序之前或之后打开。没有 reference 时可以先切入
HoloMotion，但状态只会停留在 `waiting_reference` 并继续运行 Normal；收到
11 帧连续参考后才进入 `tracking`。

## 4. 完整启动流程

日常只需要两个前台终端。Pico App 必须由操作者提前打开；工作站上的 CPU
HoloRetarget、6001 reference 发布端和 43k policy 都由 `Shift+3` 按需启动，不再
手工打开第三个 HoloRetarget 终端。

### 终端 A：启动 MuJoCo 和 BXI 控制器

```bash
cd /home/user-kevien/holomotion
source /home/user-kevien/holomotion/bxi_rl_controller/setup_holomotion_bxi.bash

ros2 launch bxi_example_py_elf3 example_launch_demo.py \
  model_file:=/home/user-kevien/bxi_rl_controller_ros2_example/src/bxi_example_py_elf3/data/mujoco_simulation/elf3.xml
```


该命令会打开 MuJoCo 窗口。正常启动时应依次看到类似日志：

```text
ELF3 state layout initialized from message names: 31 joints
state transition: ... zero_torque ... pd_brake
state transition: ... pd_brake ... normal
```

确认机器人在 MuJoCo 中保持站立，当前状态为 `com.bxi.basic_actions/normal`。

HoloMotion Mod 此时只注册状态，不加载 43k 模型、不启动 CPU HoloRetarget 服务，
也不启动 6001 接收线程。只有显式按下 `Shift+3` 后才会按需准备整条链路。

### 终端 B：启动键盘控制

必须让这个节点运行在一个可直接输入按键的前台终端中：

```bash
cd /home/user-kevien/holomotion
source /home/user-kevien/holomotion/bxi_rl_controller/setup_holomotion_bxi.bash

ros2 launch remote_controller remote_controller_keyboard.launch.py
```

不要把键盘节点放到没有焦点的后台进程中。按键前先点击终端 B，使其获得键盘焦点。

## 5. 进入和退出 HoloMotion

进入前依次确认：

1. MuJoCo 中的机器人处于 Normal 稳定站立；
2. Pico App 已开启 `Full body` 和 `Send`，PC Service 地址正确；
3. 操作者已站稳，没有坐下或处于异常骨架姿态；
4. 终端 B 已经获得键盘焦点。

然后在终端 B 按：

```text
Shift+3：进入 HoloMotion
```

首次进入会自动启动 CPU HoloRetarget systemd 服务，并按需加载、预热 43k CPU
policy，可能等待数秒。资源准备期间状态机仍在 Normal。日志顺序应为：

```text
state transition queued for resource preparation
HoloMotion policy ready: providers=('CPUExecutionProvider',)
HoloMotion status: waiting_reference
HoloMotion status: tracking
```

本机实际 provider 应为 `('CPUExecutionProvider',)`。只有看到 `tracking` 后，Pico
动作才真正进入策略。`waiting_reference` 期间每个控制周期继续运行 Normal 站立策略，
不是冻结切换瞬间的一帧电机目标，也不应把它误判为遥操已经启动。

43k 导出模型的 RoPE 长度为 8192。当前运行时会在第 8128 个策略步清空 KV cache、
将策略步号归零并继续 `tracking`，不会再因为约 164 秒达到 `rope_limit` 自动退出。
日志出现以下提示属于正常的长时间窗口轮换，不是 Pico 断连：

```text
HoloMotion RoPE step index reached 8128/8192; reset KV cache and step index while keeping tracking active
```

常用键位：

| 按键 | 功能 |
| --- | --- |
| `Shift+3` | 进入 HoloMotion |
| `1` | 返回 Normal 站立/行走 |
| `2` | Recover 恢复站立 |
| `Shift+1` | PD brake |
| `Space` | 键盘控制停止信号 |

离开 HoloMotion 后，Mod 会停止由它启动的 CPU HoloRetarget 服务，并结束专属推理
子进程、释放模型映射和 KV cache。当前工作区只借用 SONIC XML，不加载 SONIC Mod
或 SONIC policy。

## 6. 安全状态和 Reset

发生以下情况时，控制器可能返回 Normal 或进入 `zero_torque`：

- reference 超过 0.6 秒未更新；
- 连续帧队列失效；
- 机器人姿态触发 `holomotion_orientation_safety`；
- BXI 通用姿态安全检查触发 `safety`；
- 推理子进程失败或 CPU Provider 未实际启用。

如果只是回到 Normal，先重新站稳并确认 reference 连续，再按 `Shift+3`。

如果已经进入 `zero_torque`：

1. 先停止继续发送异常动作；
2. 在本地仿真中按 `2` 尝试 Recover；
3. Recover 无效或机器人已经严重倒地时，按第 8 节完整停止并重启。

不要通过关闭过期检查、绕过 11 帧启动门或无限增大超时强行进入 tracking。

MuJoCo 窗口中的原生 Reset 只重置物理模型，不等于重新启动整个 BXI 状态机。如果
Reset 后仍处于 `zero_torque`，按 Recover 或完整重启流程，不要反复点击 Reset。

## 7. 快速检查

确认所有进程：

```bash
ps -eo pid,ppid,etimes,cmd | rg \
  'simulation_mujoco|bxi_example_py_elf3_demo|holomotion_teleop_node|RoboticsServiceProcess|remote_controller --keyboard'
```

确认端口：

```bash
ss -ltnp | rg ':(6001|60061)\b'
```

正常情况下：

- `6001` 由 `holomotion_teleop_node.py` 监听；
- `60061` 由 `RoboticsServiceProcess` 在本机监听；
- ROS 节点全部位于 Domain 74；
- 未进入 HoloMotion 时，43k policy worker 不存在；
- 进入 HoloMotion 后，日志明确显示 `CPUExecutionProvider`。

查看本终端是否使用正确工作区：

```bash
echo "$ROS_DOMAIN_ID"
echo "$AMENT_PREFIX_PATH"
```

应看到 `ROS_DOMAIN_ID=74`，并且 AMENT 路径以
`/home/user-kevien/holomotion/bxi_rl_controller/install` 开头，不应包含
`/home/user-kevien/bxi_rl_controller_ros2_example/install`。

## 8. 完整停止和重新启动

推荐停止顺序：

1. 若仍能控制，在终端 B 按 `1` 返回 Normal；Mod 会自动停止它启动的 HoloRetarget 服务；
2. 在终端 B 按 `Ctrl+C` 停止键盘节点；
3. 在终端 A 按 `Ctrl+C` 停止 BXI 控制器和 MuJoCo；
4. 确认第 7 节列出的进程已经全部退出；
5. 重新按第 4 节 A、B 顺序启动。

不要同时启动两个 `holomotion_teleop_node.py`，否则会争用 6001 端口和
XRoboToolkit RoboticsService。

## 9. 常见问题

### Pico 一直显示 `waiting for body data`

依次检查 Pico App 的 `WORKING`、`Full body`、`Send`、PC Service IP 和 Tracker 校准。
这表示第一失效边界还在 Pico/XRoboToolkit 输入，尚未进入 HoloRetarget 或 policy。

### `Retarget actual=0.0Hz`

说明没有新鲜 Pico 源帧。先检查 Pico App 和网络，不要先调 policy 或 MuJoCo 参数。

### Pico 原始帧率降低，但 ZMQ 仍为 50 Hz

在 0.6 秒以内，发布端会重复处理最新有效姿态。这是抗短时抖动设计，不表示网络恢复。
若 `PicoReader` 长期只有约 10 Hz，应缩短与路由器距离或检查 Pico App 追踪状态。

### 启动时看到 `libcublasLt.so.12` 警告

先看警告属于哪个模型。Normal 的 `amp_terrain.onnx` 可能在主控制器中选择 CPU 后端；
HoloMotion 真正进入时必须看到：

```text
HoloMotion policy ready: providers=('CPUExecutionProvider',)
```

本机 HoloMotion 已固定使用 CPU。`libcublasLt.so.12` 警告若来自 Normal 的
`amp_terrain.onnx` 自动后端探测，不代表 HoloMotion 又切回了 CUDA；以上面这条
HoloMotion provider 日志为准。

### 按 `Shift+3` 没有反应

确认输入焦点在终端 B；确认只有一个键盘节点；确认终端 B 使用 ROS Domain 74。键盘
配置把 `#` 映射为 HoloMotion 事件，因此必须实际输入 `Shift+3`，不是单独按数字 `3`。

### 运行约 164 秒后自动退出 HoloMotion

这是旧版 `rope_limit` 行为，当前版本不应再发生。先确认实际运行的是独立工作区，并在
修改代码后重新构建：

```bash
echo "$AMENT_PREFIX_PATH"
rg -n "_maybe_reset_rope_window" \
  /home/user-kevien/holomotion/bxi_rl_controller/install/share/bxi_example_py_elf3/mods/com.bxi.holomotion/policy.py
```

第一条输出不应包含 GVHMR 工作区；第二条必须找到 RoPE 窗口轮换实现。不要通过增大
reference 超时来掩盖这个版本问题。

### 切入后立刻倒地

先区分三类证据：

1. `PicoReader` 或 reference 是否断流；
2. 是否进入 `tracking` 后才失稳；
3. 最终触发的是 `holomotion_orientation_safety` 还是 BXI 通用 `safety`。

如果 reference 持续 50 Hz 且已经进入 tracking 后倒地，问题不在 Pico 断流，应保存
完整控制器日志继续诊断 reference 契约、策略输出、MuJoCo 动力学或控制时序。

## 10. 当前验证边界

已确认：

- 独立 HoloMotion BXI 工作区和 GVHMR Web 工作区可以隔离运行；
- SONIC XML 能由当前 BXI 仿真加载；
- 29 维 HoloMotion 动作通过框架现有机制接入 31 actuator ELF3；
- Pico 原始输入、HoloRetarget 和 ZMQ 50 Hz 发布可以运行；
- HoloMotion Mod 按需加载 43k 模型，并能进入 `waiting_reference` 和 `tracking`；
- `0.6 s` 源数据过期阈值已在发布端和 Mod profile 对齐。
- 无 Pico reference 时在 `waiting_reference` 连续站立超过 60 秒，未触发姿态保护；
- `Shift+3` 路径可无交互密码地启动 CPU HoloRetarget 服务。
- HoloMotion/资源释放相关测试 20 项通过，真实 43k 模型文件已在 CPU ONNX Runtime
  连续执行超过 8192 步；第 8128 步轮换 KV cache 后仍保持 `tracking`。

尚未由这些检查证明：

- 当前 Pico 标定对所有操作者和动作都正确；
- 当前桌面 Linux 调度满足硬实时要求；
- MuJoCo 稳定性等同于真机稳定性；
- 这套本地仿真流程已经完成真机验证。
