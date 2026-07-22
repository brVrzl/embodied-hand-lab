# Quest 3 → JAKA MuJoCo 仿真遥操作演示

本文档对应仓库当前的正式演示链路：

```text
Quest HTS wrist/landmarks/head + CTRL v1
  -> UDP receiver / strict parsers
  -> canonical Quest frame / freshness
  -> left-index reference capture + hold-to-run
  -> filtered relative SE(3) mapping
  -> continuation IK / feasibility checks
  -> MuJoCo position-actuator plant + passive viewer
```

这是 **simulation-only** 演示。输入是 Meta Quest 3 的右手腕实时位姿，
目标模型是 JAKA Mini2 + mounted RH56 的 MuJoCo 模型；不会连接 JAKA，
不会连接 Inspire RH56DFX，也不会发送任何真机命令。这里的 6D 指 X/Y/Z
三轴平移和完整三自由度姿态变化。姿态始终通过旋转矩阵和 `xyzw` 四元数
求相对变换、滤波和误差，不直接对 roll/pitch/yaw 做差分。

## 已核实入口与边界

- 推荐入口：`./scripts/run_quest_jaka_sim_demo.sh`。
- 正式 Python 入口：`tools/quest_jaka_mujoco_sim.py live-6dof`。
- Viewer：`mujoco.viewer.launch_passive`，蓝色坐标架是 desired TCP，绿色是 simulated TCP。
- UDP receiver：`motion_input.hts_transport.HtsUdpReceiver`。
- 当前配置：`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`。
- 当前 clutch：左 Touch controller 的 index trigger 控机械臂，grip 控仿真手；host 不提供键盘 clutch 或 fallback。
- `tools/quest_jaka_hardware.py` 和所有 `scripts/run_real_*` 均不属于本演示，禁止同时启动。

当前 host 需要原 HTS wrist/landmark/head 数据和独立 `CTRL v1` sidecar。已审计的
Quest 端来源是 `brVrzl/hand-tracking-streamer` commit `5b8eac7e` 的
`feature/mixed-input-log-probe` 分支，场景内启用 `LeftControllerPacketSender`。
普通 Meta Store/上游 HTS 构建不发送 CTRL；仅有手部数据时 host 会保持
`tracking_fault`/disengaged，绝不会退回键盘控制。Quest 应用源码和 APK 不在本仓库，
因此安装正确 APK 是操作者的前置条件。

## 环境要求

当前验证主机是 Ubuntu 24.04 LTS aarch64、Python 3.12.3；项目声明 Python
`>=3.10`。推荐 Ubuntu 24.04 图形桌面。MuJoCo passive viewer 需要有效的 X11
或 Wayland 图形会话。SSH shell 通常没有 `DISPLAY`；演示脚本会只读查找同一用户的
本地 GNOME session，并继承它的 `DISPLAY/XAUTHORITY`，不会启动或修改桌面进程。

从仓库根目录创建环境：

```bash
cd "$(git rev-parse --show-toplevel)"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev]"
```

基础依赖来自 `pyproject.toml`：NumPy、MuJoCo、PyYAML；测试使用 pytest。
当前环境实际为 MuJoCo 3.9.0、NumPy 2.4.6、PyYAML 6.0.3、pytest 9.1.0。
本演示不需要 `.[sim]` 中的 ManiSkill，也不需要 JAKA SDK、ROS、serial 或 RH56 driver。

检查模型、图形会话和主机 IPv4：

```bash
test -f data/sim_assets/jaka_rh56_visual_coacd.xml
printf 'DISPLAY=%s WAYLAND_DISPLAY=%s\n' "${DISPLAY-}" "${WAYLAND_DISPLAY-}"
ip -4 route get 1.1.1.1
```

Quest 和主机必须位于互通的同一局域网，禁止 client isolation。默认使用 IPv4
UDP unicast `9000`。主机若启用 UFW：

```bash
sudo ufw allow 9000/udp
ss -lunp | rg ':9000\b'
```

`ss` 在演示启动后应看到 Python 绑定 UDP 9000。若机器有多块网卡，以
`ip -4 route get <QUEST_IP>` 显示的 `src` 地址作为 Quest 的目标主机 IP。

若自动发现失败，可从当前用户的桌面会话取得实时值，再显式追加：

```bash
./scripts/run_quest_jaka_sim_demo.sh \
  --display "${DESKTOP_DISPLAY}" \
  --xauthority "${DESKTOP_XAUTHORITY}" \
  <其余演示参数>
```

显示号可能在重启/重新登录后变化。可从桌面 shell 环境核实，而不要长期假定总是 `:1`：

```bash
GNOME_PID=$(pgrep -u "$(id -u)" -x gnome-shell | head -n 1)
eval "$(tr '\0' '\n' < "/proc/${GNOME_PID}/environ" \
  | sed -n 's/^DISPLAY=/DESKTOP_DISPLAY=/p; s/^XAUTHORITY=/DESKTOP_XAUTHORITY=/p')"
```

## Quest 端设置

1. 打开上述带 `LeftControllerPacketSender` 的 Hand Tracking Streamer 构建。
2. 目标主机填写主机局域网 IPv4；不要填 `127.0.0.1`、`0.0.0.0` 或 Quest 自己的 IP。
3. UDP 端口填写 `9000`，使用 unicast。不要启用 broadcast；host 也不会向 Quest 发起连接。
4. 开启右手 Hand Tracking。右手 wrist 和 21 landmarks 是机械臂输入与有效性来源。
5. 开启 Head Pose。它只在机械臂 index 上升沿捕获 gravity-aligned yaw；head translation、pitch、roll 和 engaged 后的头部运动均被忽略。
6. 若应用提供 Debug Info/HUD 选项，开启 Debug Info，便于携带源 sequence/timestamp；HUD 只影响 Quest 可视诊断，不是 host 控制参数。
7. 确认左 Touch controller connected、active、tracked，且扩展构建的 CTRL sender 已启用。左手裸手 tracking 不需要开启。
8. 点击 **Start Streaming**，让右手进入摄像头视野。
9. host 终端必须同时出现 `right_valid=True` 和 `controller_valid=True`。

标准 HTS UI 只配置 UDP 目的地址/端口和 hand/head/debug 流；它没有 host 端
`--bind`、`--allowed-sender`、滤波、IK 或 clutch 阈值设置。CTRL sender 使用同一目的
IP/端口。不要把尚未接入 parser 的 controller pose、键盘 clutch 或运行时 gain UI
当作可用功能。

## 主机端快速启动

先得到主机对 Quest 所在网段的 IPv4，然后显式启动所有 CLI 参数：

```bash
cd "$(git rev-parse --show-toplevel)"
source .venv/bin/activate
HOST_IP=$(ip -4 route get 1.1.1.1 | sed -n 's/.* src \([^ ]*\).*/\1/p')
DEMO_STAMP=$(date +%Y%m%dT%H%M%S)
./scripts/run_quest_jaka_sim_demo.sh \
  --config configs/sim/quest_hts_jaka_mini2_live_demo.yaml \
  --bind 0.0.0.0 \
  --port 9000 \
  --project-ip "${HOST_IP}" \
  --duration-sec 600 \
  --telemetry-hz 2 \
  --viewer \
  --report "logs/quest_jaka_sim/demo_${DEMO_STAMP}_report.json" \
  --output "logs/quest_jaka_sim/demo_${DEMO_STAMP}_raw.hts.jsonl" \
  --events "logs/quest_jaka_sim/demo_${DEMO_STAMP}_events.jsonl"
```

Raw capture 使用独占创建；上述 `DEMO_STAMP` 让每次复制执行都得到新文件且不会覆盖
旧演示证据。若无需指定文件名前缀，也可省略最后三个路径参数，入口会生成时间戳路径。

调查奇异性、IK 构型或目标跟随积压时，在同一命令追加 `--ik-debug`。它只增加 viewer/终端
诊断，不改变 target、IK 或 MuJoCo 控制：

```bash
./scripts/run_quest_jaka_sim_demo.sh <其余显式参数> --ik-debug
```

此时会显示 `q1..q6`、每帧 `delta_q`、tool-frame swing/axial roll、TCP error、Jacobian
condition/最小奇异值、安全关节限位裕量、candidate accepted/rejected、branch-switch、
hold-last、SE(3) continuation fraction/backtrack、requested backlog 和 singularity warning。
J6 contribution 仍作为几何诊断保留，但不是运动质量判据；完整值始终写入 events JSONL。

脚本只做参数展开和中文提示，最后用 `exec` 运行正式 Python 入口；没有后台 receiver
或复制的控制逻辑。关闭 viewer 或按 `Ctrl-C` 会进入 Python `finally`：receiver stop、
线程 join、socket/writer/viewer 关闭，然后写 report/events。无图形 CI 的启动检查可显式
使用 `--no-viewer`，但现场演示必须使用 `--viewer`。

## 操作流程

1. 在 Quest 中完成上一节设置并点击 Start Streaming。
2. 在图形桌面终端运行主机命令。确认首屏有 `SAFETY=...simulation only`、正确的 `PROJECT_IP`、`PORT=9000` 和 `TRANSPORT=UDP`。
3. 等待周期状态行显示 `right_valid=True controller_valid=True`。Viewer 左上角还应显示 `wrist_valid=True`、trigger 值/age 和 clutch 状态。
4. 左手 index trigger 先完全释放到 `<=0.55`。初始状态会从 `armed_waiting_for_release` 到 `disengaged`。
5. 保持右手腕和 head 有效，按住 index 到 `>=0.75`。上升沿进入 `reference_capture`，同一 control tick 捕获右手参考、head yaw 和当前模拟 TCP，随后进入 `engaged`。
6. 继续按住 index，平移右手检查 X/Y/Z；旋转右手检查三个独立姿态轴和工具 roll。蓝/绿 TCP frame 应连续跟随。
7. 释放 index：机械臂进入 `disengaged`，hold last safe target；释放期间移动右手不会累计目标。
8. 把右手移到舒适位置，再按 index：重新捕获 reference，不会跳到 Quest 的绝对坐标。
9. 可独立按住左 grip 控制仿真 RH56。grip 不决定机械臂是否 engaged；本任务不验收真手控制。
10. 正常退出：关闭 MuJoCo viewer，或在前台终端按一次 `Ctrl-C`。不要用 `kill -9`。

Reference 的意义是：按下时的过滤后 Quest wrist 为输入参考 `T_Q_H0`，同一时刻的
当前 MuJoCo TCP 为机器人参考 `T_R_P0`；后续只把相对变化复合到 `T_R_P0`。因此进入
遥操作时第一个目标严格等于当前 TCP，而不是让机器人跳到 Quest 的绝对世界坐标。

## 参数表

“代码默认值”是 parser/dataclass 的 fallback；“无（必填）”表示缺少 YAML 字段会直接
失败。演示值来自当前 live-demo YAML 或推荐脚本。共享配置只保留当前已验证的
`simulation_exploration` filter profile，不再保留物理端候选 profile 或旧的保守缩放。

### 入口、网络与状态输入

| 参数 | 当前演示值 | 单位 | 代码默认值 | 所在文件 | 作用 | 调大后的效果 | 调小后的效果 |
|---|---:|---|---:|---|---|---|---|
| `--bind` | `0.0.0.0` | IPv4 | `0.0.0.0` | `tools/quest_jaka_mujoco_sim.py` | 本地 UDP bind；不是填给 Quest 的地址 | 不适用；换具体地址会只监听该网卡 | 不适用 |
| `--port` | `9000` | UDP port | `9000` | CLI、`hts_protocol.py` | HTS 和 CTRL 共用端口 | 只会改端口，Quest 必须完全一致 | 同左 |
| `--project-ip` | 主机 LAN IPv4 | IPv4 | 自动路由探测 | CLI `_project_ip` | 打印给操作者核对；不修改 Quest | 不适用 | 不适用 |
| `--allowed-sender` | 未设置 | IPv4 | `None` | CLI、`hts_transport.py` | 可选 Quest 源 IP allow-list | 不适用 | 设置后会拒绝其他源 |
| receiver poll timeout | `0.020` | s | 固定 `0.020` | `_ReceiveWorker._run` | stop 可响应性/无包轮询；不是 tracking stale | 增大可少唤醒但退出更慢 | 减小会更频繁轮询 |
| receiver stop join timeout | `1.0` | s | 固定 `1.0` | `_ReceiveWorker.close` | 退出等待接收线程 | 增大给异常 socket 更多清理时间 | 过小可能线程尚未 join |
| `--duration-sec` | `600` | s | CLI `180` | wrapper、CLI | 有界现场时长；演示 override 给 Quest 准备留余量 | 演示窗口更长 | 到时更早正常退出 |
| `--viewer` | enabled | bool | enabled | CLI、`_viewer` | passive viewer；现场必须开启 | 不适用 | `--no-viewer` 仅诊断 |
| `--display` / `--xauthority` | 自动发现当前用户的本地 GNOME 会话，或显式传入核实值 | X11 env/path | 继承 shell | wrapper | SSH 将 viewer 投到物理显示器 | 不适用 | 不适用 |
| `--telemetry-hz` | `2` | Hz | `2` | CLI | 终端输出 `right_valid/controller_valid/arm/hand/target` | 更多日志开销/更快观察 | 更少输出；0 禁用 |
| `--ik-debug` | disabled | bool | disabled | wrapper、CLI、events telemetry | 可选显示 q/Δq、完整姿态增量、构型质量、continuation/backlog；不参与控制 | 不适用 | 不适用 |
| `input.stale_after_ms` | `250` | ms | `HtsCanonicalAssembler` 为 `250` | YAML:12、`smooth_session.py` | wrist/landmark/head freshness | 更能容忍丢包但旧数据保持更久 | 更快 fault，网络抖动更敏感 |
| `clutches.stale_after_ms` | `150` | ms | live fallback `150` | YAML:23、`live_controller.py` | CTRL freshness | 更能容忍 controller 丢包 | 更快 fail-disengaged |
| `pressed_at` / `released_at` | `0.75` / `0.55` | trigger ratio | `0.75` / `0.55` | YAML:21、`clutch.py` | analog hysteresis | press 增大需扣更深；release 增大更早释放 | press 减小更易误触；release 减小需放得更彻底 |
| `require_release_before_first_press` | `true` | bool | 状态机固有 | YAML:25、`clutch.py` | 启动/fault 后禁止 held-high 自动进入 | 不适用 | 当前实现不可禁用 |
| `head_pose_required_at_arm_capture` | `true` | bool | capture 代码固有 | YAML:11、`smooth_session.py` | reference capture 要求有效 head yaw | 不适用 | 当前 live 6D 不提供关闭路径 |
| `input_buffer_capacity` | `16` | samples | `16` | YAML:83、`smooth_session.py` | wrist 插值缓冲 | 更能覆盖抖动、占用略增 | 过小可能缺少插值样本 |
| `interpolation_delay_ms` | `20` | ms | `20` | YAML:84、`smooth_session.py` | 以小延迟换平滑时间插值 | 更平滑但操作延迟更大 | 更灵敏但抖动/外推风险更高 |

### 映射、滤波、循环与仿真

| 参数 | 当前演示值 | 单位 | 代码默认值 | 所在文件 | 作用 | 调大后的效果 | 调小后的效果 |
|---|---:|---|---:|---|---|---|---|
| `translation_scale_per_axis` | `[1,1,1]` | ratio | 无（必填） | YAML:42、`precision_mapping.py` | 精确 1:1 平移；precision mapper 明确拒绝非 1 | 不支持，会报错 | 不支持，会报错 |
| `orientation_scale(_per_axis)` | `1` / `[1,1,1]` | ratio | scale fallback `0` | YAML:46、`precision_mapping.py` | 精确 1:1 相对旋转 | 不支持，会报错 | 不支持，会报错 |
| translation basis `B_R_Y` | `[[-1,0,0],[0,0,1],[0,1,0]]` | matrix | 无（必填） | YAML:38、`precision_mapping.py` | head-horizontal 平移映射到 robot base | 不是标量，不得现场调 | 同左 |
| rotation basis `C_P_H` | `diag(-1,-1,1)` | matrix | fallback 为 translation basis | YAML:53、`precision_mapping.py` | wrist-local 到 palm-local 的 proper rotation | 不是标量，不得现场调 | 同左 |
| `translation_deadband_m` | `0.001` | m | mapping fallback `0` | YAML:43、`mapping.py` | **旧 mapper 使用；当前 precision live-6dof 不消费** | 当前演示无效果 | 当前演示无效果 |
| `orientation_deadband_deg` | `2` | deg | mapping fallback `0` | YAML:57、`mapping.py` | **旧 mapper 使用；当前 precision live-6dof 不消费** | 当前演示无效果 | 当前演示无效果 |
| `maximum_operator_displacement_m` | `0.30` | m | 无（必填） | YAML:59、`precision_mapping.py` | 当前 precision mapper 仅在 80% 时 telemetry warning；不裁剪 | warning 更晚 | warning 更早 |
| `maximum_relative_rotation_deg` | `75` | deg | mapping fallback `180` | YAML:58、`precision_mapping.py` | 当前 precision mapper 仅在 80% 时 telemetry warning；不裁剪 | warning 更晚 | warning 更早 |
| `maximum_target_displacement_m` | `0.20` | m | 无（必填） | YAML:60、`simulation.py` | 相对 initial TCP workspace envelope | 可到更远目标 | 更早 `OUTSIDE_ROBOT_WORKSPACE` |
| filter profile | `simulation_exploration` | name | 同名 | YAML:63、`smooth_session.py` | 选择当前现场验证 One Euro 参数 | 不适用 | 不适用 |
| translation One Euro `min_cutoff/beta/d_cutoff` | `1.2/18/1.0` | Hz/ratio/Hz | profile 内必填 | YAML:66、`se3.py` | position smoothing/adaptive response | cutoff/beta 大通常更灵敏、噪声更多 | 更平滑、滞后更大 |
| rotation One Euro `min_cutoff/beta/d_cutoff` | `1.5/4/1.0` | Hz/ratio/Hz | profile 内必填 | YAML:69、`se3.py` | quaternion-safe orientation smoothing | 大通常更灵敏 | 小通常更平滑/滞后 |
| `maximum_filter_dt` | `0.050` | s | profile 内必填 | YAML:72、`se3.py` | 限制异常长 filter dt | 更相信长间隔 | 更强限制停顿后的导数效应 |
| `target_generation_hz` | `60` | Hz | `60` | YAML:85、live loop | mapping + IK 固定率 | CPU 增加、目标更新更密 | 响应更离散 |
| `ik_hz` | `60` | Hz | 无独立 fallback | YAML:86 | **说明字段；live loop 实际与 target_generation_hz 同 tick** | 当前演示单独改无效果 | 同左 |
| `mujoco_control_hz` | `500` | Hz | 由 MJCF timestep 决定 | YAML:91、`simulation.step` | **说明字段**；实际用 model timestep `0.002 s` | 单改 YAML 无效果 | 同左 |
| `viewer_hz` | `60` | Hz | `60` | YAML:92、live loop | viewer sync 独立 deadline | 渲染更顺但 CPU/GPU 更高 | 画面更慢，不改变 control tick |
| `mjcf_path` | `data/sim_assets/jaka_rh56_visual_coacd.xml` | path | 无（必填） | YAML:100、`ReplayConfig` | JAKA Mini2 + mounted RH56 model | 不适用；通过 alternate config 改 | 不适用 |
| `zero_gravity` | `true` | bool | `true` | YAML:101、`simulation.py` | 演示 position plant 关闭重力 | 不适用 | false 会改变当前验证 plant |
| arm actuator `kp/kv` | `500/50` | model units | `40/0` | YAML:102、`simulation.py` | MuJoCo arm position tracking | 大更硬、更易数值振荡 | 小更软、desired/actual 滞后更大 |
| hand actuator `kp/kv` | `12/2` | model units | `8/0` | YAML:104、`simulation.py` | 仅仿真 RH56 position tracking | 大更硬 | 小更慢 |
| `integrator` | `implicitfast` | enum | `implicitfast` | YAML:106、`simulation.py` | MuJoCo 积分器；代码拒绝其他值 | 不适用 | 不适用 |
| `initial_arm_joints_rad` | `[-1.571,-0.611,-1.571,0.175,1.134,-0.262]` | rad | 无（必填） | YAML:111 | J5=65° 的已验证非奇异起姿 | 逐轴含义不同，不得整体调大 | 同左 |

### IK、连续性、hold-last 与限制

| 参数 | 当前演示值 | 单位 | 代码默认值 | 所在文件 | 作用 | 调大后的效果 | 调小后的效果 |
|---|---:|---|---:|---|---|---|---|
| `ik_gain` | `0.70` | ratio | solver `0.65` | YAML:112、`palm_target_ik.py` | DLS IK correction gain | 收敛更激进 | 收敛更慢 |
| `ik_damping` | `0.05` | ratio | `0.05` | YAML:113、solver | 奇异附近阻尼 | 更稳但精度/响应降低 | 更敏感、可能大关节步 |
| `ik_max_step_rad` | `0.04` | rad/iteration | `0.025` | YAML:114、solver | 单次 IK iteration joint step | 更快但不连续风险增 | 更稳但可能不收敛 |
| `ik_iterations` | `24` | count | `4` | YAML:115、solver | 每 target solve 上限 | 更可能收敛但 CPU 增 | 更快但拒绝可能增 |
| position/orientation tolerance | `0.0025` / `3` | m/deg | position 必填；orientation `180` | YAML:116/139、`simulation.py` | IK acceptance error | 放宽会接受误差较大解 | 收紧会增加 IK rejected |
| Jacobian condition / min singular value | `60` / `0.0125` | ratio/scaled m | 必填 | live-demo YAML、`simulation.py` | 6×6 scaled spatial **硬**奇异性 gate；与运动快慢无关 | condition 大/min singular 小更宽松 | 反向更早 `NEAR_SINGULARITY` |
| `minimum_wrist_bend_deg` | `15` | deg | `0` | YAML:124、`simulation.py` | 避免 J5≈0 时 J4/J6 counter-wind | 更保守、工作域缩小 | 更接近 spherical-wrist singularity |
| `maximum_target_jump_m/deg` | `0.04` / `8` | m/deg per tick | m 必填；deg `180` | YAML:126/140、`smooth_session.py` | 硬 Cartesian jump gate；MuJoCo demo 也用它推导单 tick SE(3) continuation 上限 | 更宽松、异常跳变风险增 | continuation 更细、backlog 可能增 |
| TCP velocity limits | `1.0` / `5.0` | m/s, rad/s | 必填 | YAML:127/128、`smooth_session.py` | 硬 target velocity gate；MuJoCo demo 在 gate 前沿同一 6D 路径分段，不再因一帧略超限形成拒绝雪崩 | 更快追上 requested target | continuation 更慢、backlog 增 |
| IK candidate joint velocity/accel | `14` / `1000` | rad/s, rad/s² | legacy keys必填 | YAML:132/133 | 病态 IK 连续性 gate，不是 actuator 命令 | 更宽松 | 更容易 IK discontinuity/accel rejection |
| command joint velocity/accel/jerk | `π` / `4π` / `20π` | rad/s, rad/s², rad/s³ | 同演示值 | YAML:134-136、`CommandTrajectoryLimits` | MuJoCo actuator setpoint 三阶整形 | 更快、更不平滑 | 更慢、更平滑 |
| command tracking frequency | `10` | rad/s | `10` | YAML:137、`jerk_limited_position_step` | 三阶 reference model bandwidth | 更紧跟 IK target | 更柔和/滞后 |
| `joint_limit_margin_deg` | `5` | deg | 无（必填） | YAML:138、IK | 关节软边界 margin | 可用范围缩小 | 更接近物理极限 |
| `maximum_joint_target_jump_rad` | `0.22` | rad/tick | `π` | YAML:143、`simulation.py` | continuation branch jump gate | 更宽松 | 更容易 `TARGET_JUMP` |
| shared SE(3) continuation | enabled | bool（共享 YAML） | enabled | `shared_target_generation`、`smooth_session.py` | MuJoCo/JAKA 共用一个比例插值 XYZ+quaternion，失败最多回退 5 次；不放宽任何 gate | 需共同审计后修改 YAML | 同左 |
| `isolated_rejection_hold_count` | 正常共享 continuation 下不用于 feasibility fault；fallback `30` | ticks | `2` | YAML、`smooth_session.py` | 仅在显式禁用共享 recovery 时生效 | 仅影响禁用 recovery 的 fallback | 同左 |
| `reject_action` | shared hold-last-safe while engaged | enum | 行为由 session 固定 | YAML、`smooth_session.py` | 无效 candidate 从不积累、不输出；tracking/controller stale 仍立即 fault | 不适用 | 不适用 |
| collision policy | reject new contacts | enum | 行为由 simulation 固定 | YAML:149、`simulation.py` | 相对初始 contact 拒绝新增 self/environment collision | 不适用 | 不适用 |
| hand reacquisition | `200` | ms | `200` | YAML:24、`clutch.py` | grip 再按时从 held command blend | 更慢更柔和 | 更快但手指 transient 增 |

当前演示仍然启用上述 workspace、jump、velocity、IK continuity、actuator
velocity/acceleration/jerk 和 collision 限制；它们不是本次新增。本文档只是准确记录
现有已验证链路，没有再加一层限制。

### 本演示禁用或不消费的参数

| 类别 | 参数/入口 | 当前状态 | 原因 |
|---|---|---|---|
| 真机专用 | YAML `hardware_adapter.*`、`jaka_transport_hz=125` | live-6dof 不消费 | 供独立 hardware entry 使用；simulation adapter 只有 MuJoCo |
| 真机专用 | `physical_mapping_confirmed=false` | 未连接/未确认 | 明确阻止把 provisional mapping 当物理标定 |
| 离线专用 | `engagement_schedule_s`、`--engage-at-sec`、fake replay cycles | live 强制空 schedule | 只允许 deterministic replay，不能伪装真实 Quest |
| 历史/旧 mapper | translation/orientation deadband | live-6dof 不消费 | precision mapper 对 filter 后完整相对 SE(3) 做 1:1 映射 |
| CLI 不存在 | `--log-level` | 不支持 | 当前日志是固定 status + JSON report/events/raw capture |
| CLI 不存在 | keyboard/reference/clutch mode switch | 不支持 | live clutch 固定为 `quest_ctrl_udp_v1`，避免静默 fallback |
| 模型/timeout override | 单独 CLI flag | 不支持 | 通过审计过的 `--config` YAML 管理，不在 wrapper 临时改写 |

## 坐标映射

HTS 原始 Unity world 是左手系：+X right、+Y up、+Z forward。
`unity_to_openxr_pose` 转为项目 `quest_world` 右手系：

```text
p_Q = (x, y, -z)
q_Q_xyzw = (-qx, -qy, qz, qw)
```

右手 wrist pose 是 `T_Q_H`。机器人/MuJoCo base frame 是 `R`，mounted RH56
palm/TCP frame 是 `P`；MuJoCo model 的 JAKA base 就是 mapping/IK 使用的 robot base，
没有额外真机 base transform。Pose 记号 `T_A_B` 表示把 B 中的向量映射到 A。

Index 上升沿捕获过滤后的 `T_Q_H0`、模拟 TCP `T_R_P0` 和 head pose。Head local
forward `-Z` 投影到 XZ gravity plane 得到 `R_Q_Y`；只锁存 yaw，不使用 head translation，
也不在 engaged 后重新采样 head。当前手腕相对变换是：

```text
delta_p_Q = p_Q_H - p_Q_H0
delta_p_Y = transpose(R_Q_Y) * delta_p_Q
delta_p_R = B_R_Y * delta_p_Y

delta_R_H = transpose(R_Q_H0) * R_Q_H
delta_R_P = C_P_H * delta_R_H * transpose(C_P_H)

p_R_target = p_R_P0 + delta_p_R
R_R_target = R_R_P0 * delta_R_P
```

其中：

```text
B_R_Y = [ -1  0  0 ]
        [  0  0  1 ]
        [  0  1  0 ]

C_P_H = [ -1  0  0 ]
        [  0 -1  0 ]
        [  0  0  1 ]
```

平移先表示到 capture 时的 head-horizontal frame，再映射到 robot base；最后只为
right-compose 转为 TCP-reference-local，代数结果仍是 `p_R_P0 + delta_p_R`，不会被
初始 TCP orientation 二次旋转。姿态保持 wrist-local body-relative delta；`C_P_H` 是绕
local Z 的 180° proper rotation，用于 Quest 右手 thumb `-X` 与 RH56 thumb `+X` 的语义对应。

IK 误差由 MuJoCo position/orientation target 与求解后 TCP 的 Cartesian norm 和
quaternion/rotation-matrix angular error计算。Continuation IK 每次从上一个 accepted
joint target 开始；candidate 通过 tolerance、singularity、jump、limit、collision 等检查后，
才形成 accepted TCP/joint target 并交给 MuJoCo adapter。拒绝的 candidate 不会变成 reference。

### 腕部机构、姿态协同与奇异性

当前 MJCF 中 J4/J5/J6 都绕各自 body-local `+Z` 转动；在演示起姿的 robot-base frame
中三轴分别约为 `[0,-0.819,-0.574]`、`[-0.985,0.100,-0.142]`、
`[-0.157,-0.858,0.489]`。J6 变换到 palm/TCP frame 后为 `[0,0,-1]`，与当前工具轴
共线但反号；J4/J5 则改变腕部方向。由于完整 6D 目标同时约束 TCP 位置和姿态，实际人手
roll/pitch/yaw 或组合平移中 J4/J5/J6 与上游关节共同运动是正常现象，不能用 J6 变化比例
判定缺陷。

对相邻目标，诊断使用四元数 `q_delta = q_swing * q_twist` 做稳定 swing–twist 分解；
它只用于解释目标，不改变 IK objective，也没有把 Quest Euler 角硬加到某个关节。
当前 DLS 是 6 关节满足 6D pose 的 continuation IK，没有可任意重分配姿态的冗余
null-space；每帧 seed 始终是上一接受解，实测日志中没有 `>=90°` branch switch。

通用 scaled-Jacobian condition `>60`、最小奇异值 `<0.0125`，或 J5 距球腕奇异点
小于 15° 时，candidate 得到 `NEAR_SINGULARITY` 并 hold last。warning 区为 condition
达到硬阈值 80%、最小奇异值低于硬阈值 1.25 倍，或 J5 距硬边界小于 5°；warning 只做
telemetry。MuJoCo demo 从 last-safe 到 requested target 使用同一个 fraction 对 XYZ 线性
插值、对 quaternion 做 shortest-path SLERP；候选失败时沿同一 6D segment 最多减半 5 次。
因此没有降级 pitch/yaw/roll，也没有放宽硬阈值。若所有回退点仍不安全，机械臂保持 last
safe、index 仍为 engaged；操作者向 reference/安全区退回后可自动恢复 accepted，无需先
release/re-capture。Controller/wrist stale 仍立即进入 `tracking_fault`。

### 奇异停止问题的量化依据（2026-07-22）

真实 Quest 仿真采集 `j6_acceptance_20260722T104047_events.jsonl` 含 4722 次 IK 尝试：
3774 accepted、948 rejected。最终用户可见的 33 次 arm fault 中，21 次显示
`TARGET_JUMP`、6 次显示 `NEAR_SINGULARITY`、另 6 次是 controller stale；但逐段回溯
27 个“连续拒绝直到 fault”的序列，首个原因实际为：13 次
`ANGULAR_VELOCITY_LIMIT`、6 次 `TARGET_JUMP`、6 次 `NEAR_SINGULARITY`、2 次
`JOINT_LIMIT`。典型序列从单帧 5–7°、略高于 5 rad/s 的正常快速手腕动作开始；首帧
被拒绝后 requested target 继续移动，而比较基准停在 last accepted，0.5 s 内累计为
30–130° 的次生 `TARGET_JUMP`。同一 clutch 内相邻 requested target 的中位/p95 为
0.24°/2.92°，只有 119/4688 帧超过 5 rad/s，说明问题是少量越门帧触发拒绝雪崩，
不是输入持续产生几十度单帧跳变。

对同一批 4722 个 requested target 做 simulation-only 离线前后回放，新 SE(3)
continuation 得到 4369 accepted、353 hard rejected：接受率从 79.9% 提升到 92.5%，
拒绝减少 62.8%；`ANGULAR_VELOCITY_LIMIT` 和 `TARGET_JUMP` 均降为 0。剩余 201 次
`NEAR_SINGULARITY`、152 次 `JOINT_LIMIT` 是目标确实到达硬边界，未通过提高阈值掩盖。
accepted candidate 的最大 condition 为 54.44、最小奇异值为 0.012518，branch switch
仍为 0，均没有越过 60/0.0125 硬门限。

起姿对比也不支持“home pose 是主因”：旧 J5=35° 起姿为 condition 12.26、
sigma-min 0.0593；当前 J5=65° 为 7.01/0.1003，且在相同离线射线测试中扩大了多个
平移/姿态方向的可用范围。J5=80° 虽有更低的起始 condition 5.98，却牺牲了部分方向
范围并改变现场已验证姿态，所以本次没有更换 home pose。

修复后的真实 Quest + viewer 验收记录 `singularity_recovery_20260722T112310` 含 4211
个输入帧、2842 次 IK、2647 accepted；187 次 `NEAR_SINGULARITY` 和 8 次
`OUTSIDE_ROBOT_WORKSPACE` 全部 hold last。共观察到 3 次同一 clutch 内从连续 hard
reject 恢复 accepted，其中最长连续拒绝 142 ticks，arm 始终保持 `engaged`；accepted
最大 condition 59.31、最小奇异值 0.012501、branch switch 0。IK candidate 的 position
error p95/max 为 0.0075/0.368 mm，orientation error p95/max 为 0.00123/0.0203°。
故“退回可恢复”和完整 6D 精度已用真实 Quest 确认。

该次刻意快速的大范围压力动作中，MuJoCo plant 相对 accepted target 的 position error
p95/max 为 76/106 mm、orientation error p95/max 为 31/64°；这是 `π rad/s` joint
command、加速度/jerk 整形与 position actuator 的可见追赶滞后，不是 IK residual，也未
触发 branch switch。本次没有为追求画面贴合而放宽硬 gate 或提高 actuator 限制；普通
演示应使用连续中等速度动作，极端快速动作的 plant lag 仍是明确限制。

## 实际状态机

机械臂真实枚举为：

| 状态 | 进入条件与允许行为 | 退出/故障与 hold-last |
|---|---|---|
| `armed_waiting_for_release` | 启动时；所有输出冻结，等待一帧有效 controller 且 index released | 有效释放后到 `disengaged`；不会因 held-high 自动捕获 |
| `disengaged` | arm frozen；释放期间 wrist motion 不积累 | 有效 index rising edge 且 wrist+head valid 时到 `reference_capture` |
| `reference_capture` | 同 tick 捕获 wrist/head yaw/current simulated TCP；该 tick first target 等于 current TCP | capture 完成到 `engaged`；capture input 无效则 fault |
| `engaged` | index 持续 pressed、controller/wrist fresh；生成 relative 6D requested target，MuJoCo 入口沿完整 SE(3) segment 连续推进 | index release 到 `disengaged`；stale/invalid 到 `tracking_fault`；feasibility rejection 仅 hold last，退回安全区可恢复 |
| `tracking_fault` | freeze/hold last safe；reference 清除；禁止自动恢复 | 必须在 fault 之后收到新鲜有效 released sample，且 capture inputs valid，回到 `disengaged`；随后重新按下/捕获 |

仿真手真实枚举为 `armed_waiting_for_release`、`disengaged`、`reacquire`、
`engaged`、`tracking_fault`。Grip rising edge 进入 200 ms `reacquire` blend；grip release
冻结最后 hand command。左 controller 整体无效会同时 fault 两通道；当前 HTS wrist 和
21-joint skeleton 是同一右手 observation，所以右手 tracking 丢失会保守地 fault 所有
正在 active 的相关通道。没有独立 `idle`、`waiting_for_tracking` 或 `stopped` enum；
进程停止是外层 loop 退出，不应把文档示例名称误写成代码状态。

短暂但未超过 stale threshold 的无新帧不会更新 target；超过 250 ms 的右手 stale 或
超过 150 ms 的 controller stale 会 fault。共享 MuJoCo/JAKA target generator 中任意
feasibility rejection 都只 hold last safe 并保留 reference，允许同一次 clutch 中退回；
tracking/controller fault 才要求 release + fresh + 再 press。物理 adapter 不另外改变该策略。

## 正常现象与验收

成功时应看到：

- 原始 capture 持续增长；终端 `right_valid=True controller_valid=True`。
- 初始 release 后 arm 为 `disengaged`；index press 后 cycle count 增加并显示 `engaged`。
- `arm_reference_pose` 非空，capture frame 无 TCP 跳变。
- 右手三轴平移改变蓝色 desired TCP，绿色 simulated TCP 连续跟随。
- 向下/向上/侧向倾斜和 wrist/tool roll 都改变蓝色 frame orientation。
- J4/J5/J6及上游关节按完整运动学连续协同；不以单关节占比作为通过条件。
- 快速一帧动作会显示 continuation/backlog 并逐步追上，不应演变为连续 `TARGET_JUMP` fault。
- 靠近硬奇异边界会 warning/hold last；保持 index 并把手退回时应恢复 accepted。
- 短暂丢帧无 target burst；长期 stale 进入 `tracking_fault` 并 hold last。
- `IK_*`、`NEAR_SINGULARITY`、`TARGET_JUMP`、workspace 或 collision rejection 不会发送异常 candidate。
- Viewer 约 60 Hz，control/IK 60 Hz，MuJoCo 500 Hz，各自 deadline；渲染 skip 不回放过期 control ticks。

现场验收清单：

- [ ] Quest 数据持续接收
- [ ] `right_valid=True`
- [ ] `controller_valid=True`
- [ ] reference 捕获成功
- [ ] XYZ 平移方向正确
- [ ] 三轴旋转方向正确
- [ ] J4/J5/J6 协同连续，无无必要的大幅构型重构
- [ ] pitch/yaw/roll 与组合 6D 均保持姿态精度
- [ ] 奇异性拒绝后 hold last，退回安全区可恢复且无构型翻转
- [ ] 无明显跳变
- [ ] 追踪丢失处理正确
- [ ] 未连接任何真机

最终 report 应包含 `mode=live_quest_to_smooth_6dof_simulation_only`，并明确
`hardware_connections=false`、`hardware_commands=false`。Events 中可检查
`right_wrist_valid`、`arm_clutch_state`、`captured_head_yaw_rad`、`operator_delta`、
`ik_status`、`accepted`、`active_arm_fault`、`actual_tcp` 和 `simulated_joint_target_rad`。

## 常见问题

### 一直 `right_valid=False`

确认开启 right hand、右手在摄像头视野、wrist 与 21 landmarks 同时发送，端口一致。
先做纯输入检查（仍不含任何机器人路径）：

```bash
PYTHONPATH=src .venv/bin/python tools/quest_hand_tracking_streamer.py live \
  --bind 0.0.0.0 --port 9000 --project-ip "${HOST_IP}" \
  --duration-sec 30 --stale-ms 250 --telemetry-hz 2
```

不要同时运行输入检查和演示；二者会争用 UDP 9000。

### 收不到 UDP / 不在同一网段

```bash
ip -br -4 addr
ip -4 route get <QUEST_IP>
ss -lunp | rg ':9000\b'
sudo tcpdump -ni any udp port 9000
```

若 tcpdump 无包，检查 Quest 目标 IP、端口、Wi-Fi client isolation 和 firewall。若有包而
host 无数据，检查 `--allowed-sender` 是否写错。

### 端口被占用

```bash
ss -lunp | rg ':9000\b'
ps -ef | rg 'quest_(jaka_mujoco_sim|hand_tracking_streamer|controller_transport_gate)'
```

先对旧前台进程按 `Ctrl-C`；不要并行启动两个 receiver。不要随意 kill 不认识的进程。

### Viewer 无法打开

```bash
printf 'DISPLAY=%s WAYLAND_DISPLAY=%s\n' "${DISPLAY-}" "${WAYLAND_DISPLAY-}"
.venv/bin/python -c 'import mujoco, mujoco.viewer; print(mujoco.__version__)'
```

从本机图形桌面终端启动。纯 SSH 需正确 X/Wayland forwarding，但现场推荐本地显示器。
若主机物理桌面已登录，先用上面的 GNOME `/proc` 命令取得 `DISPLAY/XAUTHORITY`，再传
`--display`、`--xauthority`。`--no-viewer` 只能检查加载/UDP/清理，不算现场演示验收。

### 手移动但机械臂不动

同时检查 `controller_valid=True`、arm state、index value/age。启动后必须先完全释放一次，
再按 index；只按 grip 仅控制仿真手。若显示 `tracking_fault`，先恢复 right/head/controller，
释放 index，再重新按。普通上游 HTS 没有 CTRL sidecar，永远不会 engaged。

### 平移方向反了

先确认使用本 YAML 和 `live-6dof`，且操作者朝向与已验证演示一致。Events 中检查
`operator_delta.translation_m`。不要现场改 basis 或换旧 mapping；`B_R_Y` 仍是
simulation-viewer-only provisional registration，若站位改变应单独做标定任务。

### 某个旋转方向不响应

确认 `orientation=ENABLED`、使用 `C_P_H=diag(-1,-1,1)`，查看 viewer 蓝色 frame 和 events
的 `operator_delta.orientation_xyzw`。只有 Euler UI 数值不变不能判定无响应；当前实现不用
Euler 差分。若 `ik_status` 是 orientation、singularity 或 jump rejection，先把 wrist 回到
reference 附近再测试小角度独立旋转。

### 抖动或顿挫

检查 status/events 的 wrist age、controller age、`target_skipped_ticks`、
`viewer_skipped_frames`、receive queue drops、IK time。保持当前 `simulation_exploration`
One Euro profile；不要改成旧 conservative profile。Wi-Fi 抖动优先从 packet gap 和 stale
排查，不要先加一层未经验证的限速。

### Reference capture 后跳变

Capture tick 的 desired 应等于 current simulated TCP。确认是 index **上升沿**且 head/wrist
同 tick 有效；检查 `arm_reference_pose`、`actual_tcp`、`operator_delta` 和首个 accepted event。
若入口不是 `live-6dof` 或配置不是当前 YAML，停止并换回推荐脚本。

### `IK rejected` / `tracking stale`

Events 的 `reason/ik_status/metrics` 会区分 `IK_POSITION_FAILED`、
`IK_ORIENTATION_FAILED`、`NEAR_SINGULARITY`、`TARGET_JUMP`、workspace、velocity、limit、
collision。共享 feasibility rejection 会保持 engaged/reference；看 `continuation_fraction`、
`requested_backlog_*`、`singularity_warning`，把手缓慢移回 reference 附近即可恢复。
只有 stale/invalid 才应进入 `tracking_fault`，此时按 release → fresh → press 恢复。
`right_wrist_age_s > 0.25` 或 trigger age `>0.15` 就是 stale 根因。

### 做腕部旋转时多个关节同时运动

用推荐命令追加 `--ik-debug`，先看每帧 `tool_swing_deg`、`tool_roll_deg` 和 TCP position
increment。若 swing 非零、TCP 同时平移或工具 offset 需要补偿，J4/J5/J6及上游关节共同
参与是在满足完整 6D target，并不等同于错误。判据应是 TCP error、关节连续性、condition、
最小奇异值、限位裕量和 branch switch，而不是 J6 contribution。若接近硬边界，candidate
会 `NEAR_SINGULARITY`/`JOINT_LIMIT` 并 hold last；保持 index，把手向刚才的安全方向退回，
确认状态重新 `ACCEPTED`。不要调高 condition threshold、降低限位 margin，也不要把 Quest
UI 中某个 Euler roll 直接加到任一机器人关节。

### 退出后进程未清理

```bash
ps -ef | rg 'tools/quest_jaka_mujoco_sim.py live-6dof'
ss -lunp | rg ':9000\b'
```

正常 `Ctrl-C`/关闭 viewer 会清理。若进程仍在，先向该明确 PID 发送普通 `TERM`；避免
`kill -9`，因为它无法写 report/footer。

### 如何确认没有误连接真机

- 启动行必须是 `tools/quest_jaka_mujoco_sim.py live-6dof`，不是 hardware/real 脚本。
- 终端必须打印 `SAFETY=Quest to MuJoCo only; JAKA and Inspire hardware paths are absent`。
- `ps` 中不应出现 `quest_jaka_hardware.py`、`jaka_servo_worker`、`run_real_*`、RH56 serial bridge。
- 当前 simulation import audit 禁止 `teleoperation.jaka`、`rh56_driver`、`robot_bringup`、`jkrc`、`rclpy`。
- 报告必须为 simulation-only mode 且 hardware flags 为 false。

## 参数与行为核实来源

本文交叉核实了：CLI parser、`ReplayConfig`/YAML、`HtsUdpReceiver`、canonical assembler、
CTRL parser/provider、`LatchedHeadYawArmMapper`、One Euro filters、continuation IK、MuJoCo
runner/viewer、clutch machines，以及 `test_quest_jaka_sim.py`、
`test_quest_jaka_smooth.py`、`test_precision_dual_clutch.py`、
`test_quest_jaka_wrist_roll.py`、`test_quest_live_controller_sim.py`、
`test_quest_jaka_shared_pipeline.py`。

最近相关提交是 `530d3a0`（precise dual-clutch MuJoCo teleoperation）、`9d97a8d`
（controller transport gate）和 `4a0b5e4`（provider-independent dual-clutch checkpoint）。
`QUEST_JAKA_RH56_PRECISION_DUAL_CLUTCH.md` 记录的最终 live semantic-frame session 已由
操作者确认映射正确：8,402 input frames、8 arm cycles、3,295 accepted arm targets，且
report 明确无硬件连接/命令。本页以当前代码/YAML 为准，不复用早期旧参数。
