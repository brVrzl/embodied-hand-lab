# Quest host setup

The current live path consumes two datagram families on one UDP port:

- Hand Tracking Streamer hand/head packets.
- CTRL v1 packets from `LeftControllerPacketSender`.

The host receiver timestamps each datagram, validates syntax/finiteness/order,
and keeps a bounded FIFO of 256 observations. Controller freshness defaults to
150 ms; wrist/head freshness defaults to 250 ms; input interpolation delay is
20 ms.

## Safe host checks

These commands only inspect local help:

```bash
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
```

Choose the host address explicitly; do not copy an old report's LAN address or
username. Permit the selected UDP port through the local firewall only on the
intended trusted interface. Packet capture and recording can contain personal
motion data and should follow the local data-retention policy.

## Clutch behavior

The arm uses left index trigger. The operator must release before the first
press. A rising press captures wrist, head yaw, and robot TCP references. Hold
to run; release disengages. After stale/lost input or a completed hardware
session, release and press again to capture a fresh reference.

The left grip controls RH56 only in the current simulation integration. Current
Quest-driven physical RH56 teleoperation is not validated.

The audited Unity source/build history is retained in
`docs/motion_input/QUEST_CONTROLLER_TRANSPORT_HOST.md`; it is integration
evidence, not a promise about whichever APK is currently installed.

---

# 中文版：Quest 主机设置

当前实时路径在同一个 UDP 端口接收两类 datagram：

- Hand Tracking Streamer 的手和头部数据；
- `LeftControllerPacketSender` 的 CTRL v1 控制器数据。

主机接收器为每个包记录时间戳，验证语法、有限性和顺序，并使用最大 256 条 observation
的有界 FIFO。默认控制器新鲜度为 150 ms，手腕/头部为 250 ms，输入插值延迟为 20 ms。

## 安全的主机检查

以下命令只查看本地帮助：

```bash
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
```

明确选择本机地址，不复制旧报告的 IP 或用户名。防火墙只在受信任接口上开放所选 UDP
端口。数据包记录可能包含个人动作数据，应遵循本地保留策略。

## Clutch 行为

机械臂使用左手食指 trigger。首次按下前必须先释放；上升沿捕获手腕、head yaw 和机器人
TCP 参考。按住运行，释放 disengage。输入陈旧/丢失或真机会话结束后，必须再次释放并
重新按下，捕获新的参考。

左 grip 当前只控制仿真 RH56。Quest 驱动真机 RH56 尚未验证。

Unity 源码和构建审计保存在
`docs/motion_input/QUEST_CONTROLLER_TRANSPORT_HOST.md`；它是集成证据，不保证当前头显
安装的 APK 与之相同。
