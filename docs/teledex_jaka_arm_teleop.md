# TeleDex → JAKA arm 接入与标定

状态：`iPhone pose and A/B/Toggle fields validated 2026-07-15; calibration/real JAKA pending`

本链路先做 TeleDex phone-only arm 控制，不向 RH56 发布命令。官方 TeleDex 提供 ARKit
位姿流和通用 transform 接口，但不提供 JAKA base 标定；本项目通过实测三轴运动拟合
`phone world -> JAKA base` signed axis mapping，并要求 shadow 六方向确认后才能发布实机运行命令。

## 可复用边界

| 部分 | 结论 |
|---|---|
| HEBI 设备发现、按钮/slider | 不复用；改用官方 TeleDex `Session` |
| 旧 phone→robot 映射数值 | 不信任、不作为 TeleDex 默认实机映射 |
| 相对位姿锚定、跳变拒绝、workspace、lag pause | 复用 |
| JAKA TCP/joint feedback、IK、EDG servo、watchdog | 复用 |
| RH56 hand retarget | 本阶段不启用；TeleDex landmarks 后续单独接入 |

官方 TeleDex `Session` 返回 `position`、`rotation`、`button` 和 `toggle`，README 还声明了
`button_secondary`；0.0.7 实现会漏掉第二按钮，本项目适配器已兼容保留该字段。官方
`MujocoHandler.link_body()` 支持 `scale`、origin、pre/post transform 和 position limits，
但这些只是映射机制，不是机器人坐标自动标定。参考：[TeleDex repository](https://github.com/omarrayyann/TeleDex)、
[TeleDex paper](https://arxiv.org/abs/2603.17065)。

## 1. 安装与只读验流

```bash
cd /home/thor/projects/embodied_lab
.venv/bin/python -m pip install -e ".[teledex-teleop]"
./scripts/check_teledex_phone.sh --duration-sec 15
```

脚本会显示服务器 `IP:port` 和二维码。iPhone 与 PC 应在同一 LAN；打开 TeleDex App，
扫码或手动输入地址。移动手机时应持续收到有效 pose。iOS 2.2 phone-only 页面可见
`Toggle`、`Button A`、`Button B`、`Freeze Pose`、返回和 `Reset Pose`。先执行只读字段确认：

```bash
./scripts/check_teledex_phone.sh --duration-sec 30 --controls-only
```

依次单独操作 A、B、Toggle，应分别看到 `button`、`button_secondary`、`toggle` 变化；该对应关系
已在当前 iPhone App 上实测确认。若更换 App 版本后对应关系变化，先修正配置，禁止进入实机。
该脚本不会 source ROS2，也不会发布机器人命令。

arm-only 第一阶段的控件分配：

| App 控件 | 项目行为 |
|---|---|
| Button A | 默认按住运行；松开停止，再次按下时从当前手机/机器人 pose 重新锚定 |
| Button B | 保留，不绑定机器人动作 |
| Toggle | 保留，不作为持续运行开关 |
| Freeze Pose | 仅作 App 位姿冻结功能；不作为停止保证，第一次实机不使用 |
| Reset Pose | 仅停机、退出 publisher 后重新置零；运行中不按 |
| 返回 | 断开连接，作为 Button A 之外的软停止路径 |

项目适配器额外处理了两项官方包未覆盖的安全语义：

- pose 超过 `0.20 s` 未更新或 App 断开，输出无效 snapshot 并撤销运行许可；
- TeleDex 端口已被其他进程占用时拒绝启动，避免官方 `Session` 尝试替换端口占用者。

### arm 与后续 hand 共用的安装约定

- 从 arm 阶段开始就把手机刚性安装在手/前臂下方，不再采用“屏幕朝操作者”的临时握法。
- 中立位采用掌面大致朝下：手机屏幕和前摄朝向手掌，手机背面朝向地面；手机平面、
  手掌和地面大致平行。
- 手机与手掌之间留出足够距离，使前摄能看全手掌和手指；不要把手机紧贴手掌，也不要
  遮挡前摄或背面摄像头。
- “平行地面”只定义中立位和标定起点。操作时手机必须与手/腕保持刚性相对位姿，随手腕
  一起平移和旋转，不需要始终保持水平。
- arm-only 阶段暂不使用 landmarks，但仍保持这套安装，以免进入 hand 阶段后重做外参。
- 三轴标定期间保持相同安装姿态，每次只做提示的平移，尽量不转动手机。

`Reset Pose` 是 TeleDex 的参考姿态重置，不是停止键。本项目的 relative follower 会用启动后
第一帧有效 pose 建立锚点，因此正常流程无需按它；不要在机器人运行中按。需要重新置零时，
先让机器人停止，退出 publisher/App，把手放回中立位后再重新连接。

## 2. 标定 phone-world → JAKA-base

保持 JAKA 失能；标定只读取手机：

```bash
./scripts/calibrate_teledex_jaka_frame.sh
```

按提示从同一手机原点分别沿物理 JAKA base 的 `+X`、`+Y`、`+Z` 平移至少 `0.06 m`，
尽量不旋转手机。当前项目采用 `signed_permutation`：手工移动只识别手机轴到机器人轴的
排列和正负号，最终矩阵只包含 `0/±1`，不会把自由手移动的斜向误差拟合进映射。
结果写入：

```text
configs/teleop/teledex_jaka_arm_calibration.json
```

新结果的 `real_motion_confirmed` 固定为 `false`。

## 3. RViz shadow 六方向验收

先启动机器人模型，再启动 TeleDex shadow：

```bash
./scripts/run_jaka_rh56_rviz.sh
./scripts/run_teledex_rviz_shadow.sh
```

shadow 不发布 `/jaka/teleop_palm_target_jog`，因此默认允许无外部按键预览；收到第一帧
有效 pose 后建立相对锚点，逐项验证：

```text
phone motion intended as robot +X -> marker/TCP +X
phone motion intended as robot -X -> marker/TCP -X
phone motion intended as robot +Y -> marker/TCP +Y
phone motion intended as robot -Y -> marker/TCP -Y
phone motion intended as robot +Z -> marker/TCP +Z
phone motion intended as robot -Z -> marker/TCP -Z
```

全部通过后才确认标定：

```bash
./scripts/calibrate_teledex_jaka_frame.sh \
  --confirm-for-real \
  --i-verified-shadow-six-directions
```

任一方向错误时不要确认，重新采集标定。

## 4. arm-only 实机平移

先执行 JAKA 的只读/零运动/servo capability 检查，并确认急停、使能条件、线缆和工作区：

```bash
./scripts/check_jaka_connection.sh --config configs/robot/jaka_mini2_real.yaml
./scripts/check_jaka_zero_motion.sh --config configs/robot/jaka_mini2_real.yaml
./scripts/check_jaka_edg_servo_capability.sh --config configs/robot/jaka_mini2_real.yaml
```

终端 A 启动真实 JAKA bridge；hand 使用 mock，不连接 RH56：

```bash
./scripts/run_real_arm_hand_ros2_bridge.sh \
  --hand-backend-type mock \
  --enable-arm-teleop \
  --arm-teleop-max-palm-velocity-m-s 0.06 \
  --arm-teleop-max-joint-velocity-rad-s 0.25 \
  --arm-teleop-max-joint-acceleration-rad-s2 0.80 \
  --arm-teleop-max-session-palm-excursion-m 0.10
```

终端 B 启动 TeleDex publisher：

```bash
./scripts/run_real_jaka_teledex_arm_teleop.sh \
  --enable-motion \
  --jsonl-out logs/teleop/teledex_arm_$(date +%Y%m%d_%H%M%S).jsonl
```

默认不需要 Xbox，但必须持续按住 `Button A`，同时 TeleDex pose 有效且持续更新。
松开 A 会立即发布停止并解除相对锚点；下一次按下 A 时以当时手机 pose 和机器人反馈建立
新锚点，避免松开期间的手机移动造成机器人跳变。publisher 仍必须带 `--enable-motion`，
标定文件必须已确认，bridge 也必须带 `--enable-arm-teleop`。按 App 返回/退出、断开连接或 publisher 退出后，输入适配器最多
`0.20 s` 判定 pose 失效，bridge 另有 `0.25 s` watchdog 停止 servo。两级超时最坏路径约
`0.45 s`，另加少量调度/网络延迟；它们不能替代机械急停。首次真机保持低速、低行程，
机械急停放在随手可按的位置。`Freeze Pose`、`Toggle` 和 `Reset Pose` 都不承担停止功能。

以后需要恢复独立按压门控时，可安装 `.[gamepad]`，先运行
`./scripts/check_xbox_deadman.sh --duration-sec 10`，再给 publisher 增加
`--deadman-source xbox_rb`。不要同时运行 `run_xbox_ros2_teleop.sh`，否则会向同一 arm topic
发布另一组命令。

当前配置 `orientation_control_enabled: false`，第一次真机只控制平移。待平移链路稳定后，
先在 shadow 中单轴验证 roll/pitch/yaw，再将其改为 `true`；不要直接在真机试猜测的旋转映射。

## 5. hand 阶段边界

TeleDex 官方服务端目前能暴露 `landmarks/world_landmarks`，但公开 Python 包没有针对 RH56
的 retarget。hand 阶段需要新增：landmark schema 记录、手机腕带/前摄像头验证、21 点到
RH56 canonical 六通道的 retarget、角度/速度/电流/力安全门和 mock replay。arm 验收前不启用。

## 已验证与未验证

已验证：

- TeleDex 0.0.7 本机安装、WebSocket server 启停、无手机时安全失败；
- iPhone App 到 `10.24.1.68:8888` 的真实连接；App 约 60 Hz 发帧，项目端得到
  558 个有效 10 Hz 抽样，观测数据年龄约 `0–27 ms`；
- 第二次只读控件探针得到 521 个有效抽样，并确认 `Button A -> button`、
  `Button B -> button_secondary`、`Toggle -> toggle`；
- rotation matrix → quaternion、断流撤销运行许可、非法 pose 拒绝；
- 任意 yaw frame-fit、短位移/左手系标定拒绝；
- 现有 relative follower 回归测试。
- 可选的 `pygame 2.6.1` 已安装；检查时 `joystick_count=0`，尚未接入真实 Xbox 控制器。

未验证：

- Button A 运行许可的实际 JAKA 停止时延；
- 可选 Xbox `RB` 与 TeleDex pose 的组合 shadow/实机链路；
- 实际 JAKA base 三轴标定和 RViz 六方向结果；
- 任何真实 JAKA 运动；
- TeleDex → RH56 hand retarget。
