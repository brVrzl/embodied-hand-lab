# D435 Depth Quality Assessment

> **Status: historical measurement snapshot dated 2026-07-13.** It preserves
> the observed scene and filter comparison at that time. For the current
> implementation boundary, see
> [`d435_depth_pointcloud_readiness.md`](../d435_depth_pointcloud_readiness.md).

日期：2026-07-13

## 结论

当前 top D435 深度粗糙由三类问题共同造成：

1. 原配置使用 640x480 和未固定的 Custom preset，重复性不足。
2. 原 adapter 直接在 depth 上做空间/时间滤波，没有采用 D4XX 推荐的视差域顺序。
3. 当前视野不是干净的机器人桌面工作区，而是包含笔记本、显示器、手机、鼠标、瓶子、线缆和人体的办公桌。

已采用的运行时方案：

- 848x480@30。
- Default visual preset。
- `depth -> disparity -> spatial -> depth`。
- 时间滤波默认关闭，仅静态标定/检查启用。
- 全局补洞关闭。
- 点云阶段继续使用基座坐标工作区裁剪和体素降采样。

官方依据：

- [D400 Visual Presets](https://github.com/realsenseai/librealsense/wiki/D400-Series-Visual-Presets)
- [Librealsense Post-Processing Filters](https://github.com/realsenseai/librealsense/blob/master/doc/post-processing-filters.md)
- [Librealsense releases](https://github.com/realsenseai/librealsense/releases)

## 实测对比

测试为当前相机位置下的 45-60 帧短序列。测试期间视野内有人体运动，因此结果用于选择保守配置，不作为最终标定指标。

### 分辨率

Custom preset、未滤波：

| 分辨率 | 0.3-1.5m 有效率 | 全场景时序标准差中位数 | 桌面法向噪声中位数 | 桌面法向噪声 P95 |
|---|---:|---:|---:|---:|
| 640x480 | 69.8% | 2.43 mm | 1.54 mm | 5.80 mm |
| 848x480 | 74.2% | 2.17 mm | 1.31 mm | 3.62 mm |

848x480 在当前场景的填充率和桌面噪声均优于 640x480，且与 D435 官方建议一致。

### Preset

848x480，空间+时间滤波：

| Preset | 有效率 | 桌面法向噪声中位数 | 桌面法向噪声 P95 |
|---|---:|---:|---:|
| Custom | 75.4% | 0.66 mm | 1.99 mm |
| Default | 75.4% | 0.47 mm | 1.64 mm |
| High Accuracy | 52.0% | 0.49 mm | 1.48 mm |
| High Density | 74.7% | 0.52 mm | 1.70 mm |

High Accuracy 丢失接近一半像素，不适合作为当前操作任务默认值。Default 与 High Density 接近，但 Default 更强调干净边缘和减少 point-cloud spraying，因此作为首轮运行配置。

### 滤波

848x480 High Density 短序列：

| 处理 | 有效率 | 全场景时序标准差中位数 | P95 |
|---|---:|---:|---:|
| Raw | 73.1% | 2.26 mm | 11.59 mm |
| Disparity + Spatial | 73.1% | 2.02 mm | 10.93 mm |
| + Temporal | 74.5% | 1.24 mm | 9.26 mm |
| + Global hole filling | 80.0% | 1.32 mm | 17.04 mm |

时间滤波对静态区域有效，但官方也明确说明可能产生拖影。全局补洞增加约 6.9% 像素，却把 P95 噪声推高到 17 mm；High Accuracy 场景中补洞新增约 22.5% 像素且全场景 P95 超过 50 mm。因此运行时保持关闭。

## 桌面整改要求

当前场景需要整改后再做最终参数确认：

1. 相机视野改为以 JAKA 实际操作区域为中心，尽量不包含操作者和办公区。
2. 移走工作区内的笔记本、手机、鼠标、瓶子和无关盒子；人体不要进入标定/采集 ROI。
3. 将松散线缆固定到工作区外，避免形成细长遮挡和随帧移动的深度噪声。
4. 黑色、镜面、透明物体不要作为桌面背景；建议铺固定、哑光、有轻微纹理且颜色均匀的工作垫。
5. 相机刚性固定，保留无遮挡的双红外视野和投射器窗口。
6. 清理后采集空桌面、网球、机器人进入三个场景，再确定空间滤波强度和工作区边界。

桌面不需要把整个房间整理干净，只需要保证机器人操作 ROI 及其周围约 10-15 cm 没有无关物体，并通过标定后的 3D workspace crop 排除 ROI 外背景。

## 兼容性提示

当前 SDK 为 `pyrealsense2 2.58.1`，D435 固件为 `5.15.1.55`。最新 beta SDK 2.58.2 的 D435 USB 兼容表建议固件 5.17.3.10 或更新版本；官方固件页也将 5.17.3.10 列为 2026-06 稳定版本并与 SDK 2.58.1 配套。此前设备在多次启停后出现过取帧超时；固件升级需在保持算法基线不变的独立窗口执行、备份设置并做回归，不在本次自动执行。具体顺序见 [D435 算法选择](d435_algorithm_selection_20260713.md)。
