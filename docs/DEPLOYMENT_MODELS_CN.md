# 部署模型说明

本仓库提交的是 BXI ROS2 控制器框架、Mod 代码、遥控器配置和可直接放入 Git 的模型资源。
构建产物、训练代码、训练 checkpoint 和缓存不属于部署仓库。

## 未随仓库提交的模型

`com.bxi.holomotion` 中的以下三个 ONNX 文件超过 GitHub 普通文件限制，因此没有提交：

```text
src/bxi_example_py_elf3/mods/com.bxi.holomotion/assets/model_43000.onnx
src/bxi_example_py_elf3/mods/com.bxi.holomotion/assets/native_affine_migrated_1000_20260901/model_1000.onnx
src/bxi_example_py_elf3/mods/com.bxi.holomotion/assets/native_affine_migrated_1000_20260901/model_25000.onnx
```

需要运行 HoloMotion Mod 时，把对应模型按原路径复制到目标机器，再执行 `build.sh`。
如果只使用 Normal、AMP、DWAQ、RGMT 或动作 Mod，不需要这些大模型。

仓库中的其他部署模型均小于 GitHub 单文件限制，已随代码提交。真机仍需要预装目标机器对应的 ROS 2、`bxi_ros2_pkg`、硬件驱动，以及 RGMT/HoloMotion 使用的 HoloRetarget 和 XRoboToolkit 运行依赖。
