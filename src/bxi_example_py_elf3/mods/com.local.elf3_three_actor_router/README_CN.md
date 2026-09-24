# ELF3 TienKung AMP 单策略行走 Mod（仅本地仿真）

当前 `policy.onnx` 来自 TienKung-Lab-bxi 的 model_33000 vendor handoff。
该包标记为 `UNQUALIFIED`，只允许本地离线/仿真验证，禁止部署到真机。
输入为固定 `float32[1,1020]`，输出为 `float32[1,29]`；策略直接接收
10 帧历史观测和当前速度指令，不再使用 specialist/motion/brake 三模型或
旧的横移镜像逻辑。

## 进入与退出

- 在 `normal` 中按手柄 `LB+RB+LT+X` 或键盘 `Shift+5`，通过
  `btn_10=37` 软切换进入本 Mod。
- 在本 Mod 中按手柄 `RB+X` 或键盘 `1`，软切换返回 `normal`。
- 原有手柄 `RB+B` 和键盘 `Shift+1` 仍进入 `pd_brake`。
- 首次进入需要在界面确认吊架/保护架安全提示。
- `normal`、`pd_brake` 或 `zero_torque` 事件均可退出。

## 移动输入

输入通道与 normal 模式一致：

- 键盘：`W/S` 前进/后退，`A/D` 左移/右移，`Q/E` 左转/右转。
- 手柄：左摇杆控制前后/横移，右摇杆左右控制转向。

模型仅通过下列离散单轴指令合同：前进 `+0.375 m/s`、后退
`-0.250 m/s`、横移 `±0.200 m/s`、转向 `±0.400 rad/s`。Mod 会把单轴键盘或
摇杆方向量化为对应的固定速度，因此摇杆幅度不调速。同时输入两个轴、非有限输入或
绕过 Mod 直接送入中间速度会触发 fail-closed，并请求切到零力矩。

松开移动指令后，Mod 会继续运行同一个 `policy.onnx` 并输入零速度指令；
`BRAKE`（前三秒）和 `HOLD` 只是诊断状态，不会切换到其他模型。

## 必需运行条件

- ELF3 29-DoF 本体及仓库中的对应 URDF/关节顺序。
- 50 Hz 控制周期。
- `policy.onnx` 和 `policy_manifest.json` 必须整体保留在 `assets/`。
- 状态入口只接受 `topic_prefix=simulation/`；其他前缀会 fail-closed。
- 当前交付物未通过资格门，禁止启动真机、ROS 硬件节点或电机控制。

当前源码集成和离线验证不代表真机已验收。
