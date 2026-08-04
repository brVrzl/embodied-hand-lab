# D435 Depth and Point-Cloud Readiness

日期：2026-07-13

## 审计结论

原 `vision_interface` 只能作为早期入口，不能直接支撑桌面操作：

- RGB 和 depth 依靠两次方法调用拼接，没有帧级同步契约。
- 设备时间戳、时间域和 frame number 被丢弃，只保存取帧后的主机墙钟。
- `align_depth_to_color: false` 时仍取彩色内参，深度反投影会产生系统误差。
- 双相机 YAML 没有选择单台设备的解析路径。
- 测试 fake 不含 frame profile/时间戳，无法覆盖上述错误。
- 检查工具把快照写盘时间计入采集 FPS。

这些部分已重构，旧的 `get_rgb/get_depth` 仅保留兼容用途。

## 已实现

状态 `validated by unit tests`：

- 原子 `RGBDFrame` 采集契约。
- 同时间域 RGB/depth 时间偏差门限和超限帧丢弃重试。
- 对齐/未对齐深度的正确内参选择。
- 米制深度质量统计和针孔反投影。
- 点云坐标变换、工作区裁剪和体素降采样。
- 带向上方向/倾角约束的 RANSAC 桌面拟合。
- 桌面以上指定高度范围的点提取。
- 无外参时拒绝生成伪 `jaka_base` 结果。

入口：

- `src/vision_interface/depth_processing.py`
- `tools/check_realsense_stream.py`
- `tools/process_rgbd_tabletop.py`
- `configs/perception/d435_tabletop.yaml`
- `validation/perception/test_depth_processing.py`
- `tests/test_realsense_adapter.py`

## 真实 D435 检查

设备：Intel RealSense D435，serial `346522072675`，USB 3.2，firmware `5.15.1.55`。

2026-07-13 对 top 相机做 640x480@30、彩色对齐深度、2 秒只读检查：

- 采集 60 帧，29.97 FPS。
- RGB/depth 时间戳偏差均值约 0.018 ms，最大约 0.018 ms。
- 0.15-3.0 m 有效深度 264574/307200，约 86.1%。
- 对齐深度内参：`fx=602.845, fy=602.479, cx=325.748, cy=255.930`。
- 畸变模型为 `inverse_brown_conrady`，本次系数全 0，可使用针孔反投影。

设备在连续多次启停后曾出现 `Frame didn't arrive within 5000`，SDK hardware reset 后恢复并通过上述检查。当前结论是单次采集链路可用，不等于长期运行稳定性已验证；正式采集前仍需做 10-30 分钟 soak test 和掉帧统计。

## 诊断桌面拟合

在相机坐标中对上述实拍帧做诊断拟合：

- 体素场景点 50434。
- 桌面内点 13778。
- 平面 RMSE 约 4.33 mm。
- 未使用外参和基座工作区裁剪，桌面以上点包含机械臂/支架/背景，因此不能作为物体点云质量结论。

## 尚未完成

状态 `pending`：

1. 标定 top/side D435 到 `jaka_base` 的 4x4 外参，并记录方法、日期和误差。
2. 在 `jaka_base` 中设置保守的 `workspace.min_xyz_m/max_xyz_m`。
3. 用空桌面、网球和机械臂进入视野三组场景调参，记录平面 RMSE、球点云尺寸和背景误检。
4. 按 [D435 深度质量评估](history/d435_depth_quality_assessment_20260713.md) 清理工作区后，复测运行时空间滤波和静态时间滤波。
5. 做 10-30 分钟采集稳定性测试，统计超时、掉帧、时间戳跳变和温漂。
6. 外参和工作区验证通过前，不将物体点坐标用于 JAKA 运动命令。
