# ELF3 RGMT + HoloRetarget 独立 Mod

`com.bxi.rgmt` 是独立于 `com.bxi.holomotion` 的 BXI Mod。它不导入 HoloMotion
policy、receiver、状态或模型，也不会修改 HoloMotion 的 manifest。两者只使用相同的
外部数据协议：`tcp://127.0.0.1:6001` 上的 `reference_qpos[36]`。

```text
Pico / XRoboToolkit
  -> CPU HoloRetarget
  -> reference_qpos[36] @ 50 Hz
  -> RGMT 过去10 + 当前 + 未来10 的21帧窗口
  -> rgmtr_17000.onnx: obs[1,2217] -> actions[1,29]
  -> reference joint position + residual action
  -> BXI MotorFrame
```

## 模型和输入 contract

- RGMT 模型：`assets/rgmtr_17000.onnx`
- SHA256：`e0c9ffeb1e7c0dec232859f96f6135bb5eccc5dd2f8ad9e1beddd68a5971bf14`
- Provider：固定 `CPUExecutionProvider`
- HoloRetarget 输入协议：`reference_qpos[36]`
- Observation：`21x61 command + 6D anchor + 10x93 proprio = 2217`
- Action：29维 reference residual
- Anchor：`torso_link`，四元数顺序 `wxyz`
- 控制频率：50 Hz

HoloRetarget 不直接发布关节速度和 torso 角速度，因此本 Mod 按模型的50 Hz contract
对同一个21帧 qpos 窗口做关节中心差分和四元数差分；不再加载或运行
`neural_retarget.onnx`，也不再由 RGMT Mod 直接连接 XRoboToolkit。

## 本地 MuJoCo 启动

MuJoCo 和键盘节点沿用 HoloMotion 独立工作区的启动方式。Pico App、CPU
HoloRetarget systemd unit 和 `6001` 发布协议不变。机器人在 Normal 稳定站立后，
点击键盘终端取得焦点并按：

```text
Shift+4：进入 RGMT
1：返回 Normal
2：Recover
```

单独按 `4` 仍是 AMP 奔跑。当前只新增了键盘 `Shift+4 -> btn_10=56`。

进入 RGMT 后先等待21帧连续 reference。等待期间持续运行 Normal。第一帧有效 RGMT
输出出现后才开始 `0.4 s` 线性双策略混合，混合期间 Normal 和 RGMT 都真实推进；这
避免了在21帧等待期间提前耗尽切换时间。

## 验证边界

RGMT observation/action ABI 曾与上游 `MelodyAI/bxi_elf3_ws` 的 `rgmt2.py` 对齐。
回退后仍需重新执行本地单元测试、无动作 reference 检查和 MuJoCo 闭环测试；这些不能
替代真机验证。模型 metadata 声明 `deployment_requires_recovery_latch=True`，当前没有
自行补造上游未提供的独立 latch 状态机。
