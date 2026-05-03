# vision_interface

统一 RGB-D 相机抽象层。

- 默认支持单相机接口
- 已完成：mock camera
- 占位适配：RealSense / Orbbec

topic 命名建议：

- `/sensors/camera/color/image_raw`
- `/sensors/camera/depth/image_raw`
- `/sensors/camera/color/camera_info`

frame 命名建议：

- `camera_link`
- `camera_color_optical_frame`
- `camera_depth_optical_frame`

