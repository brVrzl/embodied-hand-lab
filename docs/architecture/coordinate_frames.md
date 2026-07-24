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

Translation is expressed in the latched horizontal head-yaw frame and mapped
to robot-base coordinates with the current basis rows `[-X, +Z, +Y]`.
Configured translational gains are 1.0. Later head motion does not drag an
engaged arm reference.

Orientation remains the local/body wrist-relative rotation and is conjugated
by `diag(-1, -1, +1)` for the robot mapping. Translation and rotation then pass
through One Euro filters plus current 1 mm and 2° deadbands before continuation
IK.

Reference capture is release-before-press. A fresh controller press captures a
fresh Quest reference; stale data, stream loss, or a completed physical stop
requires release and another fresh press. Do not document SPACE-key clutch or
world-frame head-follow behavior for the current live 6-DoF path.

The generic UMIP frame contract remains documented in
`docs/motion_input/COORDINATE_FRAMES.md`; this page describes the additional
Quest/JAKA mapping policy.

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

平移先在锁存的水平 head-yaw 坐标中表达，再以当前 `[-X, +Z, +Y]` 基映射到 robot base。
平移增益为 1.0。engage 后继续移动头部不会拖动机械臂参考。

旋转保留手腕局部/body 相对旋转，并用 `diag(-1, -1, +1)` 共轭映射到机器人。平移和旋转
经过 One Euro filter，以及当前 1 mm 和 2° deadband，然后进入 continuation IK。

参考捕获采用 release-before-press。新的控制器按下捕获新的 Quest 参考；数据陈旧、流丢失
或真机 stop 后必须先释放再按下。当前 6-DoF 路径不使用 SPACE clutch，也不是实时跟随
world-frame head pose。

通用 UMIP 坐标契约见 `docs/motion_input/COORDINATE_FRAMES.md`；本页说明额外的 Quest/JAKA
映射策略。
