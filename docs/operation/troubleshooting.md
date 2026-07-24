# Troubleshooting

## No Quest engagement

- Confirm the Unity build includes CTRL v1 `LeftControllerPacketSender`.
- Confirm host address/UDP port match and the firewall allows the intended
  interface.
- Release the left index fully, then press again with fresh wrist/head packets.
- Check controller age (150 ms policy) and wrist/head age (250 ms policy).
- Do not substitute SPACE-key instructions from an old prototype.

## `HOLD_REJECTED`

This is a feasible-candidate hold, not liveness failure. Inspect the recorded
reason: IK residual, continuation, collision, branch jump, limits, Jacobian
quality, output velocity, or output acceleration. Hold still or retreat through
the last safe direction. Do not raise a boundary merely to remove the message.

J5 proximity is warning metadata; actual Jacobian quality is authoritative.

## Simulation viewer

Run without `--viewer` to separate control/replay from display problems. Over
SSH, provide the actual local `DISPLAY` and `XAUTHORITY`; never copy a stale
username or absolute path from a dated handoff.

## Native worker does not connect

Stop. Do not retry multiple SDK sessions. Confirm another client or stale
process is not logged in. The current design intentionally uses one JAKA SDK
session because a prior second-session health monitor prevented the primary
worker reaching `CONNECTED`.

## Timing, tracking, or controller fault

Treat hard timing faults, tracking errors, servo/collision alarms, estop,
power/enable loss, or SDK errors as hard stops. Preserve metrics and raw logs,
record the exact commit/config, and follow
[incident response](../safety/incident_response.md). Do not resume the same
physical envelope merely because cleanup succeeded.

---

# 中文版：故障排查

## Quest 无法 engage

- 确认 Unity build 包含 CTRL v1 `LeftControllerPacketSender`。
- 确认 host/UDP 端口一致，防火墙允许目标接口。
- 完全释放左手食指，再在 wrist/head 数据新鲜时按下。
- 检查 controller age（150 ms）和 wrist/head age（250 ms）。
- 不使用旧原型中的 SPACE 键说明。

## `HOLD_REJECTED`

这是候选不可行保持，不是活性失败。检查原因：IK residual、continuation、碰撞、分支跳变、
关节限位、Jacobian、输出速度或输出加速度。保持静止或沿最后安全方向撤退；不要为了消除
提示而提高边界。

J5 接近只属于 warning 元数据，完整 Jacobian 质量才是权威。

## 仿真 viewer

用 `--no-viewer` 把控制/回放问题与显示问题分开。通过 SSH 时传入真实本地图形会话的
`DISPLAY` 和 `XAUTHORITY`，不要复制历史文档中的用户或绝对路径。

## Native worker 无法连接

立即停止，不要重复创建多个 SDK 会话。检查其他客户端和残留进程。当前设计只允许一个
JAKA SDK 会话，因为历史上的第二监控会话会阻止主 worker 进入 `CONNECTED`。

## 时序、跟踪或控制器故障

硬时序故障、tracking error、servo/collision alarm、estop、power/enable 丢失或 SDK
错误均为 hard stop。保留 metrics 和原始日志，记录精确 commit/config，并遵循
[事故响应](../safety/incident_response.md)。清理成功并不意味着可以立即重复同一运动。
