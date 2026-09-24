# com.bxi.side_flip

侧空翻动作 Mod，使用 `DanceMotionPolicyGravityIsaaclabV3` 回放：

- `assets/side_flip.npz`：`bxi-neural-retarget export-motion` 导出的参考动作
- `assets/side_flip.onnx`：动作策略模型

从 `com.bxi.basic_actions/normal` 触发 `activate`（`btn_10=3`，默认遥控配置中的 LT+A）。动作结束后自动回到普通行走状态；`zero_torque` 可随时安全退出。
