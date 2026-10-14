# 原生观测与动作契约

本任务是 12 维总状态/动作：JAKA Mini2 六关节，随后 RH56DFX 六执行通道 `index,middle,ring,pinky,thumb_close,thumb_lateral`。不是 18 维，也不是末端位姿。前六维为关节弧度、绝对目标，不做增量积分；后六维为归一化闭合量 `[0,1]`，不是设备原始 `[0,1000]` 计数。`0` 对应默认标定 raw_open=1000，`1` 对应 raw_close=0。设备协议顺序不同，必须经过 canonical channel mapping，不能直接把数组按设备内部顺序使用。

来源：`training/pi05/openpi_adapter.py` 的 STATE_ORDER/ACTION_ORDER、源数据 `meta/info.json`、`src/rh56_driver/hand_schema.py`。实际开发样本为逻辑 episode 0 (`ep067`) 第 0 帧，其后六维为 `[0,.007,.015,.013,.127,.646]`。传感器状态来自 `rh56_angle_act` 的归一化位置读数；不是六个独立物理指节角度。模拟物理关节的耦合/拇指映射由场景实现单独负责，不由 policy worker 猜测。

## 两模型的实际训练契约

| 项目 | ACT | π0.5 |
|---|---|---|
| 本轮既定 checkpoint | nominal52/act_strong_run/100000/pretrained_model | pi05_rh56/weekend/4500 |
| 输入历史 | 当前 1 帧 | 当前 1 帧 |
| 原始 RGB | workspace/wrist，640×480 RGB uint8 | 同左 |
| 模型图像 | OpenCV INTER_AREA 320×240，CHW float32/255 | 复用训练 `Rh56Inputs` 和 OpenPI `ResizeImages(224,224)` 等比补边 |
| 相机槽 | workspace/wrist | base_0_rgb=workspace；left_wrist_0_rgb=wrist；right_wrist_0_rgb=zeros，mask=false |
| 状态与动作 | 12→12，checkpoint preprocessing/postprocessing | native12→标准 padding32→native12 slicing |
| normalization | checkpoint 自带 LeRobot processors | checkpoint/assets/local/pi05_rh56_train，quantile q01/q99 |
| 预测长度 | **60** | **16** |
| 当前动作起点 | 0 | 0 |
| delta transform | 无 | 无 |
| executor | runner 消费前8步，重新查询；不用内置动作队列 | 同左 |

π0.5 复用仓库固定 `make_config`、官方 `create_trained_policy`、实际 normalization 和模型输出 transform。已提交版本的训练 `Rh56Inputs` 要求训练 action；worker 的推理 shim 给该 transform 传入全零临时目标，随后立刻删除 `actions`，再进入 normalization/model transforms。因此模型的 observation 不含动作标签，也不会把假目标当模型输入。未采用原工作区尚未提交的224图像接入修改，以避免把已缩放画面误当原始训练画面。train/inference 的图像与状态经全部 model transforms 数值一致，见下方实测。

两模型只看当前仿真 RGB + 12D proprio；物体位置与任务阶段不进入 worker。RGB 输入保持 RGB，只有从示教视频读取开发样本时做 OpenCV BGR→RGB。ACT 原始图像压缩遵循现有 `tools/act_live_shadow.py`。π 额外图像槽为明确屏蔽的模型 padding，不是第三台相机。

## 已执行检查

本地完整证据目录：`/home/thor/projects/embodied_lab/outputs/physical_bottle_sim/policy_contract/`。原始开发 RGB 样本、权重输出及完整日志留本地，不上传。

- `transform_parity.json`：训练/推理 observation parity 通过；state `(32,)`、训练 actions `(16,32)`；三路模型图像 `(224,224,3)`；mask 为 true/true/false。native action normalize→pad→unnormalize→slice 的最大绝对误差 `6.94e-18`。
- 同一报告六个手指通道逐个端点转换：每次只将一个 canonical 通道设为1，其 raw 值变0，其余保持1000；全部 raw→normalized round-trip 最大误差0。此处验证的是接口方向，不替代场景中单关节/指节物理测试。
- `act_fixed_summary.json`：真实开发样本输出 `(60,12)`；首次422.17ms，后两次22.58/22.52ms。每次reset seed0，完整动作块差值均0。
- `pi05_fixed_summary.json`：同样本输出 `(16,12)`；首次含JIT编译34.57s，后两次286.98/238.80ms。三次seed0重置的首动作和整块范围相同；本记录未单独计算完整动作块间逐元素差值。
- 原始输出域外情况保留：ACT 手部最小值 `-0.0012903`，π0.5 手部最小值 `-0.0119727`。worker 不限幅，runner 必须保存原始与实际执行目标、投影比例。该样本域外发生在模型输出端，不能归因于仿真执行器。
- 首次 ACT 通信触发宿主 NumPy1/容器 NumPy2 ndarray pickle 兼容错误；修复为仅对 array 的 dtype/shape/bytes 编码，日志保留。π首次加载因现代 LeRobot PYTHONPATH 覆盖旧 OpenPI 数据依赖失败，修复为模型隔离的 PYTHONPATH；`pi05_worker.log` 保留该失败及后续成功记录。这些不是策略 rollout。

以上延迟为 warm 单次模型调用（含 preprocessing/postprocessing、同步转CPU），不是30Hz实测模型推理。runner 的30Hz是仿真时间频率；每8步查询意味着每次覆盖0.2667秒仿真时间，可以等待推理而不推进物理时间。

## 重置、执行与复现

`tools/bottle_policy_worker.py` 的 `BottlePolicyClient` 用独立 `--network=none` 容器加载只读原权重/代码，无设备节点挂载，无机器人控制入口。通信使用本机随机私有0700目录内的 Unix socket。ACT `reset` 清除 policy/pre/post processor状态并设PyTorch seed；π重置固定上游 policy 的JAX RNG。worker没有跨episode动作队列；runner reset时必须清空自身待执行chunk。

```python
from pathlib import Path
from tools.bottle_policy_worker import BottlePolicyClient
with BottlePolicyClient('pi05', Path('/home/thor/projects/embodied_lab'),
                        Path('/home/thor/projects/embodied-bottle-sim'),
                        Path('/home/thor/projects/embodied_lab/outputs/physical_bottle_sim/run_logs')) as policy:
    policy.reset(seed=0)
    # state: float32[12]; workspace/wrist: uint8 HWC RGB[480,640,3]
    result = policy.infer(state, workspace, wrist)
    raw_chunk = result['prediction']
```

权重路径在 client metadata 中完整记录。formal rollout 的仿真状态清单、投影限制、控制子步及阶段判定在评测协议/场景报告中记录；本文件只证明观测/动作接口与固定样本离线推理，不将它们计作抓放成功。
