# π0.5 LoRA training runbook

## English

This is training-only infrastructure for the fixed task prompt:

`Pick up the bottle and place it on the cardboard box.`

The native state/action boundary is 12-D: six JAKA arm values followed by six
RH56 active-actuator values in
`index, middle, ring, pinky, thumb_close, thumb_lateral` order. Actions are
absolute native targets. The first baseline uses two RGB images and does not
consume source force signals.

The current source/config boundary is:

- `configs/training/shared/physical_bottle.yaml`;
- `configs/training/pi05/physical_bottle.yaml`;
- `configs/training/pi05/openpi.lock.json`;
- `training/pi05/openpi_adapter.py`.

The default is `pi05_base`, official JAX, LoRA variants
`gemma_2b_lora` and `gemma_300m_lora`, action horizon 16, batch size 4,
100-step warmup, peak learning rate `1e-5`, save interval 100, and 20,000
steps. OpenPI and JAX caches, logs, and checkpoints remain under ignored
`outputs/training/pi05_rh56/`.

Check and operate the training-only supervisor:

```bash
training/pi05/scripts/check_openpi.sh --help
training/pi05/scripts/train_weekend.sh start
training/pi05/scripts/status.sh
training/pi05/scripts/train_weekend.sh stop
training/pi05/scripts/train_weekend.sh resume
```

These commands do not send robot or hand commands. The Thor container and
pinned submodule must be available for actual training.

## 中文

这是固定 task prompt 的 training-only infrastructure：

`Pick up the bottle and place it on the cardboard box.`

native state/action boundary 是 12-D：前六维 JAKA arm，后六维 RH56 active-actuator，顺序为
`index, middle, ring, pinky, thumb_close, thumb_lateral`。action 是 absolute native target。第一版 baseline
使用两路 RGB image，不读取源 force signal。

当前 source/config boundary 为：

- `configs/training/shared/physical_bottle.yaml`；
- `configs/training/pi05/physical_bottle.yaml`；
- `configs/training/pi05/openpi.lock.json`；
- `training/pi05/openpi_adapter.py`。

默认使用 `pi05_base`、official JAX、LoRA variant `gemma_2b_lora` 和 `gemma_300m_lora`，action horizon
为 16，batch size 为 4，warmup 100 step，peak learning rate 为 `1e-5`，每 100 step 保存，默认 20,000 step。
OpenPI/JAX cache、log 和 checkpoint 位于被 Git 忽略的 `outputs/training/pi05_rh56/`。

检查和运行 training-only supervisor：

```bash
training/pi05/scripts/check_openpi.sh --help
training/pi05/scripts/train_weekend.sh start
training/pi05/scripts/status.sh
training/pi05/scripts/train_weekend.sh stop
training/pi05/scripts/train_weekend.sh resume
```

这些命令不会向 robot 或 hand 发送 command。实际训练需要 Thor container 和固定版本的 submodule。
