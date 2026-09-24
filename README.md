# bxi_rl_controller_ros2_example

这是 JKYovo 维护的 BXI/ELF3 ROS 2 控制器部署仓库。它以 BXI 官方示例框架为基础，加入了本项目使用的主控框架、遥控器配置、学习策略和动作 Mod。

本仓库用于在仿真和 ELF3 真机上构建、验证和运行控制程序。训练代码、训练数据、checkpoint、构建产物和运行日志不放在这里。

## 仓库内容

- `src/bxi_example_py_elf3`：ELF3 主控、状态机、策略接口、仿真和真机 launch。
- `src/bxi_example_py_elf3/mods`：Normal、AMP、DWAQ、RGMT、HoloMotion、前/后/侧空翻等 Mod。
- `src/remote_controller`：手柄和键盘输入节点及按键映射。
- `docs`：部署、ROS 2 通信和大模型说明。

`bxi_rl_controller_ros2_example` 是控制器仓库名称；本地目录可以使用任意名称，例如 `holomotion`，不影响 ROS 2 包名。

## 依赖关系

BXI 官方基础 ROS 2 包不包含在本仓库中，真机和仿真都需要单独安装到 `/opt/bxi/bxi_ros2_pkg`。它提供通信消息、ELF3 硬件节点、MuJoCo 节点、机器人描述和相机等基础包。

```bash
sudo mkdir -p /opt/bxi
cd /opt/bxi
sudo git clone https://github.com/bxirobotics/bxi_ros2_pkg.git
```

如果目标机器已有该目录，不要重复克隆；先确认其版本和 `setup.bash` 可用。仓库中的 `bxi_ros2_pkg` 子模块声明用于记录依赖来源，不代替真机启动时使用的 `/opt/bxi/bxi_ros2_pkg`。

基础系统要求：Ubuntu 22.04、ROS 2 Humble，以及与目标机器匹配的 BXI 硬件驱动。RGMT/HoloMotion 还需要 HoloRetarget、XRoboToolkit、ZMQ 和对应 Python/推理依赖，详见各 Mod 的 README。

## 获取和构建

建议把本仓库放在独立工作区，不覆盖机器原有的官方主线工作区：

```bash
mkdir -p /home/bxi/bxi_ws
cd /home/bxi/bxi_ws
git clone https://github.com/JKYovo/bxi_rl_controller_ros2_example.git
cd bxi_rl_controller_ros2_example

git submodule update --init --recursive
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
bash build.sh
source install/setup.bash
```

`bash deploy_environment.sh` 只用于安装仓库声明的 Python 依赖。真机已有完整运行环境时不需要重复执行；执行前请确认 Python 版本和系统依赖符合目标机器环境。

构建只生成 `build/`、`install/` 和 `log/`，不会自动启动主控、硬件或电机。

## 仿真启动

在已经 source 上述环境的终端中运行：

```bash
ros2 launch bxi_example_py_elf3 example_launch_demo.py
```

手柄节点：

```bash
ros2 launch remote_controller remote_controller.launch.py DEBUG:=true
```

没有手柄时可以使用键盘 launch。仿真验证通过后再使用硬件 launch。

## 真机启动

启动真机前先确认机器人处于安全状态，并确认没有其他主控进程：

```bash
pgrep -af 'bxi_example_py_elf3_demo|hardware_elf3|example_demo_hw'
```

在第一个终端加载环境并启动主控和硬件：

```bash
cd /home/bxi/bxi_ws/bxi_rl_controller_ros2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch bxi_example_py_elf3 example_launch_demo_hw.py
```

该 launch 会启动硬件节点、相机节点和 `bxi_example_py_elf3_demo` 主控。确认主控日志、硬件通信和 IMU 状态正常后，再在第二个终端启动遥控器：

```bash
cd /home/bxi/bxi_ws/bxi_rl_controller_ros2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch remote_controller remote_controller.launch.py DEBUG:=true
```

如果 `ros_elf_launch.service` 已经运行遥控器，先检查它的 `ExecStart` 和工作区路径，不要同时启动第二个遥控器。普通仓库更新不修改、替换或新增机器上的自启动服务；要使用本仓库作为开机启动目标，应按现场流程单独配置并验证。

主控和遥控器是两个独立进程。遥控器显示 `active` 不代表主控已经启动；应同时看到主控进程和状态机加载日志。按键是否生效以 `src/remote_controller/config/xbox_default.yaml` 和实际安装后的 `install/share/remote_controller/config/xbox_default.yaml` 为准。

## 模型说明

仓库中已提交可放入普通 Git 的部署模型。3 个超过 GitHub 单文件限制的 HoloMotion ONNX 没有上传：

- `com.bxi.holomotion/assets/model_43000.onnx`
- `com.bxi.holomotion/assets/native_affine_migrated_1000_20260901/model_1000.onnx`
- `com.bxi.holomotion/assets/native_affine_migrated_1000_20260901/model_25000.onnx`

使用 HoloMotion Mod 前，需要把对应文件按原路径复制到机器，再重新构建。详见 [部署模型说明](docs/DEPLOYMENT_MODELS_CN.md)。

## 安全和维护

- 不要在已有主控运行时覆盖源码或执行构建。
- 不要同时启动两个硬件主控或两个遥控器节点。
- 真机先在仿真验证，启动硬件前确认急停和安全操作员就位。
- 出现通信超时、IMU 错误、过扭矩或控制周期停止时，立即停止后续操作并保存日志。
- `hw` launch 会访问真实硬件，不能当作仿真命令使用。

更多说明：

- [ELF3 新动作部署教程](docs/ELF3_NEW_ACTION_DEPLOYMENT_CN.md)
- [ELF3 ROS 2 通信与运行架构](docs/ELF3_ROS2_COMMUNICATION_GUIDE_CN.md)
- [部署模型说明](docs/DEPLOYMENT_MODELS_CN.md)
