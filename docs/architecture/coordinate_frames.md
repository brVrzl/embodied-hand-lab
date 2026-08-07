# Quest-to-JAKA coordinate frames

The incoming Hand Tracking Streamer data uses Unity/OpenXR conventions and is
canonicalized by `motion_input.hts_canonical` before robot mapping. Do not add
another implicit handedness or quaternion-order conversion downstream.

At clutch capture the session stores:

- the current wrist transform;
- a gravity-aligned head-yaw basis;
- the authoritative robot TCP pose associated with the accepted/held joints.

For each current wrist sample, the local relative transform is:

```text
T_relative = inverse(T_wrist_reference) * T_wrist_current
```

Translation is expressed in the latched horizontal head-yaw frame. For the
current operator-aligned setup, OpenXR right/up/forward map to robot-base
`-Y/+Z/+X`; equivalently the basis rows are `[-Z, -X, +Y]`. Configured
translational gains are 1.0. Later head motion does not drag an engaged arm
reference.

Orientation remains the local/body wrist-relative rotation and is conjugated
by `diag(-1, -1, +1)` for the robot mapping. Translation and rotation then pass
through One Euro filters plus current 1 mm and 2° deadbands before continuation
IK.

Reference capture is release-before-press. A fresh controller press captures a
fresh Quest reference; stale data, stream loss, or a completed physical stop
requires release and another fresh press. Do not document SPACE-key clutch or
world-frame head-follow behavior for the current live 6-DoF path.

The provider-level UMIP frame contract is summarized in the Motion Input
platform page. This page is the single current authority for both the generic
input convention and the additional Quest/JAKA mapping policy.

## UMIP provider convention

UMIP poses are child-frame poses relative to the sample's explicit
`coordinate_frame`. Providers emit right-handed metres and `x,y,z,w` unit
quaternions. They perform only the documented source-basis conversion; robot
registration, calibration, filtering, scaling, IK, and safety remain downstream
responsibilities. The Quest frame identity includes device, session, and
reference-space IDs, so coordinates from different sessions are never silently
combined.

---

# 中文版：Quest 到 JAKA 坐标系

Hand Tracking Streamer 输入使用 Unity/OpenXR 约定，并先由
`motion_input.hts_canonical` 规范化。下游不得再次隐式转换手性或四元数顺序。

捕获 clutch 参考时，session 保存：

- 当前手腕变换；
- 与重力对齐的 head-yaw 基；
- 与已接受/保持关节对应的机器人 TCP 权威位姿。

当前手腕样本的局部相对变换为：

```text
T_relative = inverse(T_wrist_reference) * T_wrist_current
```

平移先在锁存的水平 head-yaw 坐标中表达。当前操作者与机器人同向时，OpenXR 的
右/上/前分别映射到 robot-base `-Y/+Z/+X`，等价矩阵各行为 `[-Z, -X, +Y]`。
平移增益为 1.0。engage 后继续移动头部不会拖动机械臂参考。

旋转保留手腕局部/body 相对旋转，并用 `diag(-1, -1, +1)` 共轭映射到机器人。平移和旋转
经过 One Euro filter，以及当前 1 mm 和 2° deadband，然后进入 continuation IK。

参考捕获采用 release-before-press。新的控制器按下捕获新的 Quest 参考；数据陈旧、流丢失
或真机 stop 后必须先释放再按下。当前 6-DoF 路径不使用 SPACE clutch，也不是实时跟随
world-frame head pose。

UMIP pose 是相对于 sample 显式 `coordinate_frame` 的子坐标系 pose。provider 输出右手系米制
坐标和 `x,y,z,w` 单位四元数，只做已记录的源坐标基转换；机器人注册、标定、滤波、缩放、IK
和安全仍由下游负责。Quest frame identity 包含 device、session 和 reference-space ID，不同
session 的数值坐标不能被静默合并。本页同时是通用输入约定和 Quest/JAKA 额外映射策略的唯一
当前权威页。
