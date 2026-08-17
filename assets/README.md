# Simulation assets

## English

The default mounted MuJoCo asset is `jaka_rh56_visual_coacd.xml`. Its source
model is `jaka_rh56.xml`; the runtime asset uses the reviewed visual/CoACD
geometry and current RH56 actuator limits. The committed asset is consumed by
the simulation configs and is not rebuilt automatically during runtime.

Use the asset builder only for offline inspection or the explicit check:

```bash
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py --check
```

`assets/correll_rh56dfx/` is a licensed reference model, not the mounted
JAKA/RH56 runtime. Preserve upstream attribution and do not treat reference
geometry as physical collision authority.

## 中文

默认挂载的 MuJoCo asset 是 `jaka_rh56_visual_coacd.xml`，源模型是 `jaka_rh56.xml`；runtime asset 使用经过
review 的 visual/CoACD geometry 和当前 RH56 actuator limit。提交的 asset 由 simulation config 使用，运行时不会
自动重建。

asset builder 只用于离线检查或显式 check：

```bash
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py --check
```

`assets/correll_rh56dfx/` 是带 license 的 reference model，不是挂载的 JAKA/RH56 runtime。保留 upstream
attribution，不要把 reference geometry 当作真机 collision authority。
