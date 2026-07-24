# Configuration

Versioned YAML is loaded from `configs/`; command-line arguments select
environment-specific values such as bind address, duration, and physical
authorization. Precedence for a current entry point is:

```text
code schema/defaults < selected YAML < explicit CLI override
```

Do not assume every older tool uses that complete hierarchy; verify its
`--help` and loader.

The authoritative Quest live policy is
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`. Related files:

- `configs/motion_input/quest_hts_right_hand.yaml`: HTS provider/input facts;
- `configs/sim/quest_hts_jaka_mini2_offline.yaml`: recorded/offline simulation;
- `configs/sim/quest_rh56_retarget.yaml`: simulated hand retargeting;
- `configs/robot/jaka_mini2_real.yaml`: physical connection example, not
  authorization or controller truth.

Local IPs, serial devices, display paths, and camera URLs belong on the command
line or in uncommitted local configuration. Example addresses are not a
statement of the current network. Payload/TCP/controller settings are not
owned by these files.

---

# 中文版：配置

版本化 YAML 从 `configs/` 加载；绑定地址、时长和真机授权等环境相关值由命令行选择。
当前入口的优先级为：

```text
代码 schema/默认值 < 选定的 YAML < 显式 CLI 覆盖
```

不要假设每个旧工具都实现了完整层级；应核对它的 `--help` 和配置加载代码。

Quest 实时策略的权威配置是
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`。相关文件包括：

- `configs/motion_input/quest_hts_right_hand.yaml`：HTS provider/输入事实；
- `configs/sim/quest_hts_jaka_mini2_offline.yaml`：录制输入/离线仿真；
- `configs/sim/quest_rh56_retarget.yaml`：仿真灵巧手重定向；
- `configs/robot/jaka_mini2_real.yaml`：真机连接示例，不代表授权或控制器真实配置。

本地 IP、串口设备、显示路径和相机 URL 应放在命令行或未提交的本地配置中。示例地址
不表示当前网络状态。这些文件不负责管理 payload、TCP 或控制器设置。
