# Quest CTRL host transport

## English

`tools/quest_controller_transport_gate.py` is the current input-only host gate.
It parses strict `CTRL,v=1,...` packets alongside legacy HTS hand/head lines,
preserves session and sequence diagnostics, and reports staleness using host
monotonic receive time. It does not start MuJoCo, generate a robot target, or
open a JAKA/RH56 device.

The CTRL parser rejects missing, duplicate, unknown, or reordered fields,
invalid integers/booleans, non-finite analog values, malformed UTF-8, and
trailing content. Arm and hand clutch facts are independent:

| Fact | Press | Release |
| --- | ---: | ---: |
| `index` | `>= 0.75` | `<= 0.55` |
| `grip` | `>= 0.75` | `<= 0.55` |

After startup, restart, malformed input, or staleness, both channels require a
valid released observation before a new press edge. Run only as an input gate:

```bash
PYTHONPATH=src .venv/bin/python tools/quest_controller_transport_gate.py \
  --bind 0.0.0.0 --port 9000 --project-ip "$HOST_IPV4" \
  --print-hz 5 --required-data-timeout-sec 20 --duration-sec 180
```

## 中文

`tools/quest_controller_transport_gate.py` 是当前 input-only host gate。它在 legacy HTS hand/head line
旁解析严格的 `CTRL,v=1,...` packet，保留 session/sequence diagnostics，并使用 host monotonic receive time
计算 staleness。它不会启动 MuJoCo、生成 robot target 或打开 JAKA/RH56 设备。

CTRL parser 会拒绝缺失、重复、未知或乱序字段，非法 integer/boolean、非 finite analog value、错误 UTF-8
和尾随内容。arm 与 hand clutch fact 独立：

| Fact | Press | Release |
| --- | ---: | ---: |
| `index` | `>= 0.75` | `<= 0.55` |
| `grip` | `>= 0.75` | `<= 0.55` |

启动、restart、malformed input 或 stale 后，两路都必须先观察到有效 release，再允许新的 press edge。只把它作为
input gate 运行：

```bash
PYTHONPATH=src .venv/bin/python tools/quest_controller_transport_gate.py \
  --bind 0.0.0.0 --port 9000 --project-ip "$HOST_IPV4" \
  --print-hz 5 --required-data-timeout-sec 20 --duration-sec 180
```
