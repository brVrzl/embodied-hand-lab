# quadruped_adapter

云深处机器狗统一适配层，当前不绑定具体型号。

接口：

- `connect()`
- `get_robot_state()`
- `get_odom()`
- `teleop(cmd_vel)`
- `stand()`
- `sit()`
- `estop()`
- `start_recording_hint()`

真实设备接入时优先修改：

1. `interfaces.py`
2. `adapter.py`
3. 新增 `deeprobotics_backend.py`

若不同型号 topic/service/SDK 不同，仅替换 backend，不要改 recorder 或任务层。

