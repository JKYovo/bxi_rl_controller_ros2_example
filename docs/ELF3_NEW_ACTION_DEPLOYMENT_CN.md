# ELF3 新动作真机部署教程

本文适用于在 `bxi_rl_controller` 中新增一个学习策略动作，或把现有 Mod 的模型替换成新模型，然后部署到 ELF3 真机。

这里的“动作”指一个由 ELF3 主控加载的策略 Mod，例如 AMP、DWAQ 或其他具有 `plugin.py`、`policy.py`、`state.py` 和 `mod.yaml` 的模块。

## 0. 先区分两种情况

### 只替换同一策略的模型

如果新模型和旧模型完全使用相同的：

- 观测维度和排列；
- 历史帧数量和排列方向；
- 动作数量和关节顺序；
- 默认关节位置；
- action scale、KP、KD；
- 控制频率和速度指令单位；

可以复用现有 Mod，只替换模型资源，并修改入口中的模型文件名（如果文件名发生变化）。

### 新增一个不同策略的动作

如果上述任意一项不同，应建立新的 Mod。不要把新模型直接替换进 AMP 或 DWAQ，否则模型可能能够加载，但收到的是错误的观测或关节动作。

## 1. 先固定训练端契约

部署前从训练代码和导出脚本确认以下内容，并写入部署代码或 `policy.json`：

| 项目 | 必须确认的内容 |
| --- | --- |
| 单帧观测 | 维度、每一段的含义和排列顺序 |
| 历史观测 | 帧数、最旧到最新还是相反顺序 |
| 动作 | 数量、关节顺序、输出范围 |
| 关节参数 | 默认姿态、action scale、KP、KD |
| 速度指令 | `vx`、`vy`、`wz` 的单位和范围 |
| 周期 | 训练控制周期和部署推理频率 |
| 相位 | 是否包含相位、相位顺序和推进方式 |

用 ONNX Runtime 检查导出文件的输入和输出 shape，并确认一次无动作推理能够得到有限值。不要只根据文件名或训练 README 猜测输入契约。

## 2. 准备完整 Mod

新 Mod 通常放在：

```text
src/bxi_example_py_elf3/mods/com.example.new_action/
```

至少包含：

```text
com.example.new_action/
├── mod.yaml
├── plugin.py
├── policy.py
├── state.py
├── README.md
└── assets/
    ├── policy.onnx
    └── policy.json
```

`policy.json` 不是所有训练导出都提供；如果提供，应让部署代码校验它，而不是只校验 ONNX tensor shape。

`mod.yaml` 至少要核对：

- 唯一的 `id` 和 `entrypoint`；
- `enable: true`；
- `requires` 和 `conflicts`；
- 状态名称、优先级和 `inference_hz`；
- 速度配置和限幅；
- 激活事件值；
- 从 `normal`、`pd_brake`、新状态返回 `normal` 或 `zero_torque` 的路由。

新 Mod 必须使用没有冲突的事件值。事件值在遥控器配置和 `mod.yaml` 中必须完全一致。

## 3. 本地构建和仿真

在本地工作区执行：

```bash
cd ~/holomotion/bxi_rl_controller
source /opt/ros/humble/setup.bash

# 如果本机安装了 BXI 二进制包，还要 source 它的环境
source /opt/bxi/bxi_ros2_pkg/setup.bash

colcon build --packages-select bxi_example_py_elf3 \
  --symlink-install --merge-install
source install/setup.bash
```

如果同时修改了 `remote_controller`，一起构建：

```bash
colcon build --packages-select bxi_example_py_elf3 remote_controller \
  --symlink-install --merge-install
source install/setup.bash
```

启动仿真：

```bash
ros2 launch bxi_example_py_elf3 example_launch_demo.py
```

仿真中至少验证：

1. 主控正常启动；
2. 日志出现新 Mod 的 `loaded`；
3. 状态图中存在新状态；
4. `normal -> 新动作` 可以切换；
5. 新动作能够稳定站立；
6. 前进、后退、横移、转向分别测试；
7. 新动作能够返回 `normal` 或 `pd_brake`；
8. 输入输出频率和推理耗时没有明显超时。

仿真通过只说明软件链路和策略行为基本正确，不能证明目标真机的总延迟、总线和电机执行一定稳定。

## 4. 配置遥控器按键

遥控器源码配置在：

```text
src/remote_controller/config/xbox_default.yaml
```

如果新动作复用已有按键，不要新增映射，也不需要重启遥控器节点。

如果要新增按键：

1. 选择一个未使用的 `btn_10` 值；
2. 在 `xbox_default.yaml` 中添加手柄组合到该值的映射；
3. 在新 Mod 的 `mod.yaml` 中使用同一个值；
4. 保留已有映射，不要覆盖 AMP、DWAQ、RGMT 或 HoloMotion 的值；
5. 构建 `remote_controller`，让安装目录中的配置同步更新。

检查安装配置：

```bash
ros2 pkg prefix remote_controller
```

然后确认以下文件包含新映射：

```text
<remote_controller prefix>/share/remote_controller/config/xbox_default.yaml
```

只修改 `src/remote_controller/config/xbox_default.yaml` 而不重新构建，运行中的节点可能仍然读取旧的 `install/share` 配置。

## 5. 部署前检查目标机

先登录目标机并只做只读检查。以下路径用目标机实际工作区替换：

```bash
cd <target workspace>
ros2 pkg prefix bxi_example_py_elf3
ros2 pkg prefix remote_controller

systemctl cat ros_elf_launch.service
systemctl show -p ExecStart ros_elf_launch.service
ps -ef | grep -E 'bxi_example_py_elf3|remote_controller' | grep -v grep
```

重点确认：

- 当前 source、build、install 路径；
- `ros_elf_launch.service` 实际启动的程序；
- 遥控器设备路径是否真实存在；
- 当前是否已有主控进程；
- 目标机 Python、NumPy、ONNX Runtime 和相关动态库是否可用。

不要为了部署新 Mod 修改、替换或新增自启动服务，也不要把本地仿真的 launch 文件写进真机 systemd unit。

## 6. 同步完整 Mod 并在目标机构建

只更新 Mod 时，同步完整的 Mod 目录：

```bash
rsync -a \
  src/bxi_example_py_elf3/mods/com.example.new_action/ \
  <user>@<robot>:<target workspace>/src/bxi_example_py_elf3/mods/com.example.new_action/
```

不要只同步 ONNX 或 `plugin.py`。如果修改了公共框架、`setup.py` 或主控代码，也要同步相应源码文件。

在目标机重新构建：

```bash
cd <target workspace>
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash

colcon build --packages-select bxi_example_py_elf3 \
  --symlink-install --merge-install
source install/setup.bash
```

如果遥控器配置也修改了：

```bash
colcon build --packages-select remote_controller \
  --symlink-install --merge-install
source install/setup.bash
```

构建后确认安装产物中同时存在：

```text
install/share/bxi_example_py_elf3/mods/com.example.new_action/mod.yaml
install/share/bxi_example_py_elf3/mods/com.example.new_action/plugin.py
install/share/bxi_example_py_elf3/mods/com.example.new_action/assets/...
```

同时记录模型 SHA256，避免目标机实际运行的是旧模型：

```bash
sha256sum <target workspace>/src/bxi_example_py_elf3/mods/com.example.new_action/assets/policy.onnx
sha256sum <target workspace>/install/share/bxi_example_py_elf3/mods/com.example.new_action/assets/policy.onnx
```

## 7. 什么时候重启遥控器和主控

### 只替换模型

遥控器按键没有变化时：

- 不需要重启遥控器节点；
- 需要重启主控，确保旧的策略实例和 ONNX 后端被释放并重新加载。

### 修改按键配置

需要：

1. 重新构建 `remote_controller`；
2. 重启已有遥控器服务或重新启动原来的遥控器 launch；
3. 检查手柄已被打开；
4. 检查按键事件值；
5. 再重启主控加载新 Mod。

重启后遥控器日志应先出现：

```text
open joystick
initial state ready
```

按键测试时应出现：

```text
remote btn changed: btn_10=XX
```

`ros_elf_launch.service` 显示 `active` 只能说明服务进程存在，不能代替上述日志检查。

## 8. 主控加载验证

主控必须使用目标机原有的真机启动方式。不要用本地仿真命令替代真机启动服务。

主控重启后按以下顺序检查：

1. 日志出现新 Mod 的 `loaded`；
2. 状态图中包含新状态；
3. 模型预热或按需加载没有报错；
4. 触发按键后，主控日志出现状态转换；
5. 状态转换目标是新 Mod，而不是只出现遥控器事件。

只有遥控器日志：

```text
remote btn changed: btn_10=XX
```

不能证明动作已经切换。必须同时看到主控状态机的 transition。

## 9. 真机首次测试

真机测试前确认：

- 机器人处于安全姿态或保护架内；
- 急停可立即使用；
- 操作员能马上切回 `pd_brake` 或 `zero_torque`；
- 没有重复运行的主控实例；
- IMU、硬件节点和电机通信正常。

首次测试顺序建议为：

1. 进入新动作但不发送明显速度指令；
2. 观察站立和控制周期；
3. 小幅前进；
4. 小幅后退；
5. 小幅横移；
6. 小幅转向；
7. 切回 PD；
8. 查看 IMU、过扭矩、通信超时和 deadline miss。

如果出现电机超时、IMU 错误、过扭矩、控制周期停止或快速振荡，应立即回安全态并保存日志，不要连续重启尝试。

## 10. 常见故障定位

| 现象 | 优先检查 |
| --- | --- |
| 没有 `remote btn changed` | 手柄设备、按键组合、遥控器安装配置、遥控器是否重启 |
| 有按键事件，没有状态转换 | `mod.yaml` 事件值、Mod 是否安装、主控是否重启、`install/share` 是否为新文件 |
| 有 `loaded`，切换时报模型错误 | ONNX 输入输出、`policy.json`、关节顺序、依赖和 ONNX Runtime |
| 服务 active，但没有 `open joystick` | systemd 环境、设备路径、动态库和节点实际进程 |
| 仿真正常，真机抖动 | 端到端延迟、周期抖动、推理耗时、总线和电机执行延迟 |
| 新动作影响其他动作 | 事件值冲突、状态路由冲突、Mod 常驻资源或错误的 `conflicts` 配置 |

## 11. 部署记录

每次部署至少记录：

```text
目标机器：
目标工作区：
Mod id：
模型文件：
模型 SHA256：
源码/构建/install 路径：
构建命令：
遥控器事件值：
遥控器设备路径：
主控实际 ExecStart：
主控日志中的 Mod loaded：
主控日志中的 state transition：
是否启动过电机：
真机测试结果：
未验证项目：
```

核心原则是：本地仿真通过后，**同步完整 Mod，在目标机重新构建，检查 install 产物，再分别验证遥控器事件和主控状态转换**。普通 Mod 更新不需要修改自启动服务。
