# ELF3 DWAQ Mod

This Mod packages `elf3_model_28400_export` from the DWAQ deployment export.
`policy.onnx` consumes five oldest-to-newest 100-dimensional observations and
returns 29 actions. The observation includes the 0.8 s alternating gait phase;
the joint order and per-joint gains/scales match `assets/policy.json`.

On the Xbox/BattleDragon mapping, `LB+RB+LT+Y` emits `btn_10=38` and enters
the Mod from the normal or PD-brake state. The normal remote velocity controls
are clipped to the training envelope: `vx [-0.6, 1.0]`, `vy [-0.5, 0.5]`, and
`yaw [-1.57, 1.57]`.

The policy resource is on-demand, so the ONNX session is created only when the
operator requests this state. Use the existing zero-torque and PD-brake
controls to leave the state during commissioning.
