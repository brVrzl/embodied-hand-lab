# Data collection

## English

Collection is review-first and offline by default. The canonical writer stores
episode metadata, synchronized RGB frames, robot state/action, timing, and
quality information. A staging episode must be reviewed and approved before
conversion to a training-eligible dataset.

The current collection profile is selected in
`configs/data_collection/physical_collection.yaml`. `production` keeps the
canonical row and bounded diagnostics; `diagnostic` is for short commissioning
checks and is not a training filter.

### Start the maintained physical collection

The combined teleoperation wrapper is the collection entry. It owns the Quest
input, JAKA/RH56 control, dual RealSense capture, and `lerobot_staging_v1`
episode recorder in one process boundary:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config configs/data_collection/physical_collection.yaml
```

The current production configuration writes raw episodes under
`data/raw_episodes/`, uses explicit camera serials, records at 30 Hz, and
disables preview/commissioning streams. The arm-only wrapper is not a normal
dataset-collection entry because it intentionally does not command RH56.
Review [Real-device operation boundary](../operation/REAL_ROBOT.md) and
[Real-hardware safety](../safety/REAL_HARDWARE_SAFETY.md) before any physical
run. The command duration and device paths come from the YAML; do not append a
second ad-hoc recorder or change the robot safety configuration.

After collection, inspect and synchronize a staging episode before approval:

```bash
.venv/bin/embodied-lab dataset inspect data/raw_episodes/<episode-id>
.venv/bin/embodied-lab dataset validate data/raw_episodes/<episode-id>
.venv/bin/embodied-lab dataset sync-staging data/raw_episodes <episode-id> \
  --camera-tolerance-ms 100 --output outputs/sync/<episode-id>.json
.venv/bin/embodied-lab dataset review-staging data/raw_episodes <episode-id>
.venv/bin/embodied-lab dataset approve-staging data/raw_episodes <episode-id> \
  --status approved --notes "reviewed by <operator-id>"
.venv/bin/embodied-lab dataset convert-staging data/raw_episodes <episode-id> \
  outputs/staging_converted/<episode-id>
```

Raw episodes are immutable. Approval/conversion creates derived data and does
not repair or overwrite the source capture.

Current read-only dataset commands are exposed by `embodied-lab dataset`:

```bash
.venv/bin/embodied-lab dataset validate <episode-directory>
.venv/bin/embodied-lab dataset inspect <episode-directory>
.venv/bin/embodied-lab dataset manifest <dataset-root> <manifest.json>
.venv/bin/embodied-lab dataset statistics <manifest.json> <statistics.json>
.venv/bin/embodied-lab dataset sync-staging <root> <episode-id> \
  --camera-tolerance-ms 100 --output sync_check.json
```

Synchronizers are causal: they never backfill a future camera, control, force,
or timestamp sample into an earlier row. Missing/stale inputs are recorded as
quality faults. Raw/master inputs are immutable; derived views are written to
new output roots.

## 中文

数据采集默认先审核并离线进行。canonical writer 保存 episode metadata、同步 RGB frame、robot
state/action、timing 和 quality 信息。staging episode 必须人工 review 和 approve 后，才能转换为
training-eligible dataset。

当前 collection profile 在 `configs/data_collection/physical_collection.yaml` 中选择。`production`
保留 canonical row 和有界 diagnostics；`diagnostic` 只用于短时 commissioning 检查，不能作为训练过滤器。

### 启动维护中的真机采集

联合 teleoperation wrapper 就是采集入口；它统一管理 Quest 输入、JAKA/RH56 控制、双 RealSense
以及 `lerobot_staging_v1` episode recorder：

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config configs/data_collection/physical_collection.yaml
```

当前 production 配置使用显式 camera serial、30 Hz 记录，把 raw episode 写入 `data/raw_episodes/`，并关闭
preview/commissioning stream。仅机械臂 wrapper 不发送 RH56 command，因此不是正常 bottle 数据采集入口。
真机运行前阅读[真机边界](../operation/REAL_ROBOT.md)和[真机安全](../safety/REAL_HARDWARE_SAFETY.md)。command
duration 和 device path 来自 YAML；不要额外启动第二个 recorder，也不要修改 robot safety 配置。

采集后先 review 和同步，再 approve staging episode：

```bash
.venv/bin/embodied-lab dataset inspect data/raw_episodes/<episode-id>
.venv/bin/embodied-lab dataset validate data/raw_episodes/<episode-id>
.venv/bin/embodied-lab dataset sync-staging data/raw_episodes <episode-id> \
  --camera-tolerance-ms 100 --output outputs/sync/<episode-id>.json
.venv/bin/embodied-lab dataset review-staging data/raw_episodes <episode-id>
.venv/bin/embodied-lab dataset approve-staging data/raw_episodes <episode-id> \
  --status approved --notes "reviewed by <operator-id>"
.venv/bin/embodied-lab dataset convert-staging data/raw_episodes <episode-id> \
  outputs/staging_converted/<episode-id>
```

raw episode 不可变；approve/convert 只生成派生数据，不会修复或覆盖源采集。

当前只读数据命令由 `embodied-lab dataset` 提供：

```bash
.venv/bin/embodied-lab dataset validate <episode-directory>
.venv/bin/embodied-lab dataset inspect <episode-directory>
.venv/bin/embodied-lab dataset manifest <dataset-root> <manifest.json>
.venv/bin/embodied-lab dataset statistics <manifest.json> <statistics.json>
.venv/bin/embodied-lab dataset sync-staging <root> <episode-id> \
  --camera-tolerance-ms 100 --output sync_check.json
```

同步器遵循因果顺序：不会把未来 camera、control、force 或 timestamp sample 倒填到更早 row。缺失或过期
输入会记录为 quality fault。raw/master 输入不可变；派生 view 写入新的 output root。
