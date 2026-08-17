# System architecture

## English

The maintained control path has one shared arm-target boundary:

```text
Quest HTS/CTRL or replay
  -> protocol validation and bounded input state
  -> release-before-press clutch/reference capture
  -> frame mapping and filters
  -> continuation IK and feasibility checks
  -> immutable AcceptedArmTarget
  -> MuJoCo adapter or JAKA accepted-joint adapter
```

MuJoCo and JAKA receive the same accepted six-joint arm target. The physical
adapter does not read MuJoCo `qpos`, remap frames, filter, or solve IK. The
native joint worker makes zero JAKA `kine_inverse` calls.

The RH56 path is separate from arm target generation. It exposes six active
actuator channels in this order:
`index, middle, ring, pinky, thumb_close, thumb_lateral`.
Raw feedback and force-register values retain their source meaning; they are
not silently converted into passive-joint or calibrated tactile state.

The data path is review-first: raw episodes are finalized, validated and
split at episode boundaries before derived ACT, ACT+Force, LeRobot, or OpenPI
views are produced. Source rows and videos are not rewritten by materializers.

## 中文

维护中的控制管线只有一个共享 arm-target boundary：

```text
Quest HTS/CTRL 或 replay
  -> 协议校验和有界输入状态
  -> release-before-press clutch/reference capture
  -> frame 映射和滤波
  -> continuation IK 和 feasibility 检查
  -> 不可变 AcceptedArmTarget
  -> MuJoCo adapter 或 JAKA accepted-joint adapter
```

MuJoCo 和 JAKA 接收同一个已接受的六关节 arm target。物理适配器不会读取 MuJoCo `qpos`、重新映射
frame、滤波或重新求 IK。native joint worker 不会调用 JAKA `kine_inverse`。

RH56 管线与 arm target 生成分开，六个 active actuator channel 的顺序为：
`index, middle, ring, pinky, thumb_close, thumb_lateral`。原始 feedback 和 force-register 值保留
源语义，不会静默转换成 passive-joint 或 calibrated tactile state。

数据管线以 review-first 为边界：raw episode 先 finalize、validate，并在 episode boundary 上切分，
再生成 ACT、ACT+Force、LeRobot 或 OpenPI view。materializer 不会重写源 row 和 video。
