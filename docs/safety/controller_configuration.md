# Controller configuration boundary

The controller is the authority for payload, center of mass, installation,
TCP, collision settings, speed limits, and other safety parameters. Repository
configuration may document expectations, but the Quest/JAKA path must not
write these values automatically.

Latest operator-supplied record:

- payload 0.8 kg;
- COM `[9.289, 12.427, 36.961]` mm;
- upright/floor installation, X=0°, Z=0°;
- TCP1–TCP10 zero;
- controller safety limits unchanged.

Before a future authorized physical test, read/confirm the actual controller
state through the approved procedure. Do not “correct” a mismatch in software
or at the controller without a separate engineering decision and explicit
authorization.

TCP remaining zero is a known limitation: it is not proof of calibrated
tool-center geometry. Do not present pose accuracy at an uncalibrated tool
frame as a completed TCP validation.

---

# 中文版：控制器配置边界

控制器是 payload、质心、安装方向、TCP、碰撞设置、速度限制和其他安全参数的权威。仓库
配置可以记录期望值，但 Quest/JAKA 路径不能自动写入这些值。

最近一次操作者提供的记录：

- payload 0.8 kg；
- COM `[9.289, 12.427, 36.961]` mm；
- upright/floor 安装，X=0°，Z=0°；
- TCP1–TCP10 全零；
- 控制器安全限制未更改。

未来经过授权的真机测试前，必须通过批准流程读取/确认真实控制器状态。没有单独工程决策和
显式授权时，不得在软件或控制器中“修正”不一致。

TCP 仍为零是已知限制，并不证明工具中心已经标定。不得把未标定工具坐标系下的位姿精度描述
为已完成 TCP 验证。
