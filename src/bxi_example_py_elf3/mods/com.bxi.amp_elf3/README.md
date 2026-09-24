# ELF3 AMP V4 部署命令说明

本模块直接使用主控已接收的 `ctx.current_raw_cmd_vel`，不再二次订阅
`motion_commands`，也不依赖 ROS graph 中“恰好只有一个发布者”的条件。

## V4 速度命令

当前训练配置按物理速度输入，部署端只做限幅：

- `vx = clip(raw_vx, -0.6, 1.0)` m/s
- `vy = clip(raw_vy, -1.0, 1.0)` m/s
- `wz = clip(raw_wz, -1.0, 1.0)` rad/s；纯转向按
  `turning_max_abs_ang_vel=2.0` 允许到 ±2 rad/s，平移时限制为 ±1 rad/s。

策略观测 `6:9` 直接接收限幅后的物理速度，不除以最大速度，不增加 heading。
平移仍经过原有 `0.98/0.02` 滤波，因此摇杆映射值是目标值。

heading 仍是训练内部目标，不加入部署 command。
