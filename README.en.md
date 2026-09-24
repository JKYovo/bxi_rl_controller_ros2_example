# bxi_rl_controller_ros2_example

This is JKYovo's BXI/ELF3 ROS 2 controller deployment repository. It contains the controller framework, remote-control configuration, policy modules, and motion Mods used by this project.

Training code, datasets, checkpoints, build outputs, and runtime logs are intentionally kept out of this repository.

## Contents

- `src/bxi_example_py_elf3`: ELF3 controller, state machine, policy interfaces, and simulation/hardware launch files.
- `src/bxi_example_py_elf3/mods`: Normal, AMP, DWAQ, RGMT, HoloMotion, and motion Mods.
- `src/remote_controller`: joystick/keyboard input node and bindings.
- `docs`: deployment, ROS 2, and model notes.

The repository name remains `bxi_rl_controller_ros2_example`. The local checkout directory may have another name, such as `holomotion`.

## Dependencies

The official BXI ROS 2 base packages are maintained separately. Install them on the machine at `/opt/bxi/bxi_ros2_pkg`; they provide communication messages, ELF3 hardware, MuJoCo, robot descriptions, and camera nodes.

```bash
sudo mkdir -p /opt/bxi
cd /opt/bxi
sudo git clone https://github.com/bxirobotics/bxi_ros2_pkg.git
```

Ubuntu 22.04, ROS 2 Humble, and the hardware-specific BXI drivers are required. RGMT/HoloMotion additionally require HoloRetarget, XRoboToolkit, ZMQ, and their Python/inference dependencies.

## Clone and build

Use a separate workspace so the machine's existing upstream workspace remains untouched:

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

Building only creates `build/`, `install/`, and `log/`; it does not start the controller, hardware, or motors.

## Simulation

```bash
ros2 launch bxi_example_py_elf3 example_launch_demo.py
ros2 launch remote_controller remote_controller.launch.py DEBUG:=true
```

Verify a policy in simulation before using the hardware launch.

## Hardware

Before starting, confirm the robot is safe and no other controller is running:

```bash
pgrep -af 'bxi_example_py_elf3_demo|hardware_elf3|example_demo_hw'
```

Start the hardware controller in one terminal:

```bash
cd /home/bxi/bxi_ws/bxi_rl_controller_ros2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch bxi_example_py_elf3 example_launch_demo_hw.py
```

This starts the hardware node, camera nodes, and `bxi_example_py_elf3_demo`. After the controller logs and hardware communication are healthy, start the remote controller in a second terminal:

```bash
cd /home/bxi/bxi_ws/bxi_rl_controller_ros2_example
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch remote_controller remote_controller.launch.py DEBUG:=true
```

If `ros_elf_launch.service` is already running, check its `ExecStart` and workspace path before starting another remote-controller process. Normal repository updates do not modify machine startup services.

The controller and remote input are separate processes. A running remote-controller service alone does not prove that the main controller is running. Check both the controller logs and state-machine transition logs. Verify bindings in `src/remote_controller/config/xbox_default.yaml` and the installed copy under `install/share/remote_controller/config/`.

## Models

Three HoloMotion ONNX files exceed GitHub's regular per-file limit and are not included. Copy them to their original paths before using the HoloMotion Mod. See [deployment model notes](docs/DEPLOYMENT_MODELS_CN.md).

## Safety

- Do not build over an active controller workspace.
- Do not run two hardware controllers or two remote-controller nodes at once.
- Verify in simulation before using any `*_hw` launch file.
- Stop immediately on communication timeouts, IMU errors, torque faults, or a stopped control loop.

More documentation:

- [ELF3 action deployment guide](docs/ELF3_NEW_ACTION_DEPLOYMENT_CN.md)
- [ELF3 ROS 2 communication guide](docs/ELF3_ROS2_COMMUNICATION_GUIDE_CN.md)
- [Deployment model notes](docs/DEPLOYMENT_MODELS_CN.md)
