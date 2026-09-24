# HoloMotion 本地模型选择与回退清单

更新时间：2026-09-01（Asia/Shanghai）。

## 当前选择

```text
native_affine_migrated_1000_20260901/model_1000.onnx
SHA256 dd13111af1a421453e6af653ecb8599356a75ee4c5f02b1000e8ad786ae1f958
```

训练谱系由随包 `onnx_contract.json` 声明为：

```text
model_44000 actor
-> exact affine migration
-> new critic and optimizer
-> iteration 1000
```

该版本的完整随包文件保存在：

```text
assets/native_affine_migrated_1000_20260901/
```

## 回退基线

```text
model_43000.onnx
SHA256 be7103032b3eadb249d04e4d2faba123dd27e11861e97e7d96a34ea7bf7a0dda
```

该文件与以下 checkpoint 资产是同一硬链接，部署切换不得覆盖它：

```text
/home/user-kevien/holomotion/checkpoints/Holomotion-elf3/model_43000.onnx
```

回退时只修改 `plugin.py` 的 `model_path` 为：

```python
resource.asset("assets/model_43000.onnx")
```

然后重新构建 `bxi_example_py_elf3` 并执行 CPU 无控制加载检查。

## 已删除版本

旧的 `model_native_action_1000.onnx` 是用户主动删除的退化版本，不恢复、
不作为回退候选。
