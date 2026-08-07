# Physical test gates

Physical progress is monotonic and evidence-based:

1. offline build/static/unit/replay validation;
2. read-only and no-motion checks;
3. plant-free/fake-worker transport validation;
4. bounded operator-initiated physical operation;
5. evidence review before any larger envelope.

Every gate records branch/commit, config and executable identity, controller
state, selected operation mode, bounds, start/end state, stop reason, metrics, raw-log
relationship, and explicit PASS/FAIL/partial status. A pass applies only to the
tested path and envelope.

Historical Gates 1–3C are indexed under
`docs/history/gates/jaka_foundation_20260716/`. Gate 3C includes successful
minimal J6 motion for its exact historical procedure; it does not validate the
current full Quest pipeline.

The next recommended Quest/JAKA gate is described in
`docs/status/current_status.md`. Repository maintenance does not execute or validate this physical operation.

---

# 中文版：真机测试 Gate

真机进度必须单调、以证据为依据：

1. 离线构建、静态检查、单元测试和回放；
2. read-only 和 no-motion 检查；
3. plant-free/fake-worker 传输验证；
4. 受限的真机运行；
5. 审阅证据后才允许扩大运动范围。

每个 gate 都应记录 branch/commit、配置和可执行文件身份、控制器状态、运行模式、边界、
起止状态、停止原因、metrics、原始日志关系和明确的 PASS/FAIL/partial。PASS 只适用于该路径和该范围。

历史 Gates 1–3C 在 `docs/history/gates/jaka_foundation_20260716/`。Gate 3C 的最小 J6
运动只验证其历史精确流程，不证明当前完整 Quest 管线。

当前推荐的 Quest/JAKA gate 见 `docs/status/current_status.md`。运行维护命令或阅读本页不会打开、连接或控制真实设备。
