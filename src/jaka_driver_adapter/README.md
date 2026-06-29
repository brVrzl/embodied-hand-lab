# jaka_driver_adapter

JAKA mini2 的统一接口封装层。这里不重写官方驱动，也不伪造官方 API。

- 已完成：统一接口、mock backend、基于本地官方 `jkrc` Python SDK 的 real backend 骨架
- 本地参考资料：
  - `third_party/jaka_sdk/v2.2.7/linux/python3/aarch64-linux-gnu/jkrc.so`
  - `third_party/jaka_sdk/v2.2.7/linux/python3/x86_64-linux-gnu/jkrc.so`

当前 real backend 直接对接这些已在本地资料中确认过的方法名：

- `login_in`
- `login_out`
- `power_on`
- `power_off`
- `enable_robot`
- `disable_robot`
- `get_joint_position`
- `get_tcp_position`
- `joint_move`
- `linear_move`
- `motion_abort`

注意：

- `move_pose()` 中 pose 参数编码方式是依据本地 C++ 文档推断出的 Python 绑定常见形式，真实接入时应先用你本机安装的 `jkrc` 绑定做一次实测确认。
- `set_speed_scale()` 优先尝试 `set_rapidrate`，若当前 SDK 版本没有该接口，会退化为只在 adapter 内记录速度比例。
