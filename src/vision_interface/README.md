# vision_interface

统一 RGB-D 相机抽象层。

- 默认支持单相机接口
- 已完成：mock camera / RealSense
- 占位适配：Orbbec

RealSense D435 本地检查：

```bash
python3 -m pip install -e '.[realsense]'
python3 tools/check_realsense_stream.py --duration-sec 3 --width 640 --height 480 --fps 30
```

topic 命名建议：

- `/sensors/camera/color/image_raw`
- `/sensors/camera/depth/image_raw`
- `/sensors/camera/color/camera_info`

frame 命名建议：

- `camera_link`
- `camera_color_optical_frame`
- `camera_depth_optical_frame`
