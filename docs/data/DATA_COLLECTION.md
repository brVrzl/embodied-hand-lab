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
