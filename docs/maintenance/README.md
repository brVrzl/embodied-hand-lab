# Maintenance / 维护说明

This directory contains only current maintenance policy. Historical cleanup
logs and validation reports are kept in [`../../dev_tmp/maintenance/`](../../dev_tmp/maintenance/)
and are not current operating instructions.

本目录只保留当前维护规则。历史清理日志和验证报告统一放在
[`../../dev_tmp/maintenance/`](../../dev_tmp/maintenance/)，不作为当前操作说明。

## Current maintenance rules / 当前维护规则

- Preserve unrelated user changes, protected hardware tools, models, captures,
  and calibration data.
- Keep one authoritative page per topic; put dated evidence in `dev_tmp/` or
  `docs/history/`.
- Validate commands from the repository root and label offline, simulation,
  replay, and physical evidence literally.
- Never weaken hardware boundaries or delete startup continuity, watchdog,
  limit, collision, timing, or cleanup checks during cleanup.

- 保留无关用户改动、受保护的真机工具、模型、采集数据和标定数据。
- 每个主题只保留一个权威页面；带日期的证据放入 `dev_tmp/` 或 `docs/history/`。
- 从仓库根目录验证命令，并如实标注离线、仿真、回放和真机证据等级。
- 清理时不得削弱真机边界，也不得删除启动连续性、看门狗、限位、碰撞、时序或清理检查。
