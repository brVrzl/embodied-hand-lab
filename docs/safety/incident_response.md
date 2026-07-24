# Incident response

On collision/servo alarm, estop, tracking fault, timing fault, unexpected
motion, or communication failure:

1. stop commanding and allow the existing cleanup path to finish;
2. do not immediately reconnect, re-enable, or retry the same motion;
3. record controller-visible alarms, robot/command state, timestamps, exact
   commit, config, executable, acknowledgement, and operator observations;
4. preserve raw logs without editing them;
5. reproduce offline or with the fake worker where possible;
6. add/fix regression coverage before proposing another bounded physical gate;
7. state whether the cause is proven, suspected, or unresolved.

The July 2026 Quest/JAKA incident sequence is preserved under
`docs/history/incidents/quest_jaka_20260722_23/`. It includes a J4 collision
alarm, payload correction reported by the operator, a failed two-session
health-monitor attempt with no motion, the later sole-session timing run, and
the offline output-acceleration correction. The historical outcomes must not be
rewritten as a single PASS.

---

# 中文版：事故响应

出现 collision/servo alarm、estop、tracking fault、timing fault、意外运动或通信失败时：

1. 停止命令，让现有 cleanup 完成；
2. 不立即 reconnect、re-enable 或重复同一运动；
3. 记录控制器报警、机器人/命令状态、时间戳、精确 commit、配置、可执行文件、授权和
   操作者观察；
4. 保留原始日志，不编辑原始证据；
5. 尽可能离线或用 fake worker 复现；
6. 在提出下一受限真机 gate 前增加/修复回归测试；
7. 明确原因是已证明、推测还是未解决。

2026 年 7 月的 Quest/JAKA 事故序列保存在
`docs/history/incidents/quest_jaka_20260722_23/`，包括 J4 collision alarm、操作者报告的
payload 修正、无运动的双会话健康监控失败、后续单会话时序运行和离线输出加速度修复。
不得把这些历史结果合并改写成一次完整 PASS。
