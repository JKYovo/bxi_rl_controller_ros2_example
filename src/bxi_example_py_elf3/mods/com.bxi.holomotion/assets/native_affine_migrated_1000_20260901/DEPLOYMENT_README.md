# HoloMotion ELF3 native-action model 1000 deployment package

This package contains the corrected affine-migrated ELF3 29-DoF policy saved
at fine-tuning iteration 1000.

## Files

- `model_1000.onnx`: deployment policy with KV-cache inputs and embedded robot
  metadata.
- `config.yaml`: fully composed training contract used to create the model.
- `onnx_contract.json`: concise deployment-facing copy of the ONNX I/O and
  robot action contract.
- `SHA256SUMS`: integrity hashes for all three files above.

The PyTorch checkpoint, critic, optimizer, datasets, and robot assets are not
required for policy inference and are intentionally excluded.

## Required runtime behavior

1. Treat ONNX metadata as authoritative for `joint_names`, `action_scale`,
   `default_joint_pos`, `joint_stiffness`, and `joint_damping`.
2. Map robot observations into the exact ONNX `joint_names` order.
3. Interpret policy output using:

   `q_target = default_joint_pos + action_scale * raw_action`

   The three waist entries have signed scales because the ELF3 policy and
   simulator/controller axes differ. Do not take their absolute values and do
   not apply the previous G1 affine bridge again.
4. Store the previous ONNX raw output as `last_action`. Do not convert it to
   joint radians before placing it in the next policy observation.
5. The policy uses the HoloMotion v1.4 KV-cache interface. Reset the cache and
   `step_idx` at the beginning of a new episode/reference stream.
6. Run the deployment runtime's no-action preflight before enabling control.

An older ELF3 deployment runtime is compatible only if it follows the rules
above. A runtime with hard-coded G1/old-ELF3 scale, offset, gains, joint order,
or converted `last_action` must be updated before this model is used.

This package has not itself been validated on a real robot.
