# vision_interface

RGB-D 采集和桌面几何处理模块。核心路径不依赖 Open3D，单位统一为米。

## 数据契约

新代码应调用 `CameraInterface.capture()`，一次取得同一 frameset 的 `RGBDFrame`：

- `rgb`：`uint8 [H,W,3]`，RGB 顺序。
- `depth_m`：`float32 [H,W]`，单位米，0 表示 D435 无效深度。
- `intrinsics`：始终对应 `depth_m` 的像素几何。深度对齐到彩色时是彩色内参，否则是深度内参。
- 主机时间、彩色/深度设备时间戳、时间域和两个 frame number。
- `depth_aligned_to_color`：决定 RGB 是否能直接赋色给深度点云。

当两个设备时间戳属于同一时间域时，adapter 默认丢弃偏差超过 50 ms 的启动帧，最多重试 30 帧；门限可通过 `max_timestamp_skew_ms` 和 `sync_retry_frames` 调整。不同时间域的数值不直接相减。

`get_rgb()` / `get_depth()` 只作为旧调用兼容层保留。数据记录和感知代码不要用两个独立调用拼一帧。

RealSense 光学坐标为 `x` 向右、`y` 向下、`z` 向前。`depth_to_point_cloud()` 不会猜测机器人坐标，也会拒绝带非零畸变系数却未矫正的图像。

## D435 采集检查

```bash
python3 -m pip install -e '.[realsense]'
.venv/bin/python tools/check_realsense_stream.py \
  --duration-sec 3 \
  --width 848 --height 480 --fps 30 \
  --serial 346522072675 \
  --snapshot-dir data/reports/realsense_top
```

输出包含：

- 深度有效率和距离分位数。
- RGB/深度设备时间戳偏差。
- 实际深度对应内参和畸变信息。
- `realsense_rgb.npy`、`realsense_depth_m.npy`。
- `realsense_point_cloud.npz` 和 `realsense_frame.json`。

`configs/camera/realsense_thor.yaml` 是双相机配置，代码中必须明确选择 `side` 或 `top`：

```python
from vision_interface import RealSenseCamera

with RealSenseCamera.from_yaml(
    "configs/camera/realsense_thor.yaml",
    camera_name="top",
) as camera:
    frame = camera.capture()
```

桌面运行配置使用 Default preset 和视差域空间滤波。时间滤波默认关闭，避免机械臂和物体运动残影；全局补洞关闭，避免桌面深度跨越物体边缘。静态标定时可单独启用时间滤波。

浏览器预览提供三种模式：

```bash
# 运行时检查：仅空间滤波
.venv/bin/python tools/serve_realsense_viewer.py --depth-filter-profile spatial

# 空桌面/静态标定：增加时间滤波
.venv/bin/python tools/serve_realsense_viewer.py --depth-filter-profile static
```

`static` 不用于移动中的机器人/物体。深度显示默认固定为 0.3-1.5 m，无效像素显示为黑色，可用 `--depth-min-m/--depth-max-m` 调整。

生成同帧算法和点云对比报告：

```bash
.venv/bin/python tools/compare_d435_depth_algorithms.py \
  --serial 346522072675 \
  --output-dir data/reports/d435_algorithm_compare
```

报告比较 Raw、视差域 Spatial、静态 Temporal 和实验性 RGB guided fill；guided fill
不能仅凭有效率提升进入几何主链。

## 点云和桌面处理

`depth_processing.py` 提供：

- 深度质量统计和带范围/mask/stride 的反投影。
- 4x4 齐次变换、轴对齐工作区裁剪、体素降采样。
- RGB 引导的小孔补全实验接口，以及统计/半径点云离群剔除。
- 带向上轴和最大倾角约束的 RANSAC 平面拟合。
- 桌面平面与指定高度范围内的物体点提取。

离线处理 D435 检查工具保存的一帧：

```bash
.venv/bin/python tools/process_rgbd_tabletop.py \
  --depth data/reports/realsense_top/realsense_depth_m.npy \
  --rgb data/reports/realsense_top/realsense_rgb.npy \
  --metadata data/reports/realsense_top/realsense_frame.json \
  --target-from-camera path/to/top_target_from_camera.npy \
  --output-dir data/reports/tabletop_top
```

`target_from_camera` 必须是从相机光学坐标到配置中 `target_frame_id` 的 4x4 变换。没有外参时工具默认拒绝桌面任务输出。仅调试平面可显式使用：

```bash
--allow-camera-frame --camera-up-axis X Y Z
```

这种输出会标记 `diagnostic_camera_frame_only: true`，不可用于 JAKA 目标位置。

算法参数在 `configs/perception/d435_tabletop.yaml`。外参完成后还必须填写 `workspace.min_xyz_m/max_xyz_m`，否则桌面以上点会混入机械臂、支架和背景。

## 当前边界

- 原子帧、点云和合成桌面分割已有单元测试。
- top D435 已完成真实 RGB-D/点云只读检查。
- camera-to-`jaka_base` 外参和基座工作区边界尚未标定。
- 因此外参后的真实物体点云和抓取目标仍是 pending，不能直接下发给机器人。
