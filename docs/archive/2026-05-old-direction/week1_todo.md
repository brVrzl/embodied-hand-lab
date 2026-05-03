# Week 1 Todo

目标：不要急着训练。第一周只证明 JAKA mini2 + RH56 能稳定采出 fixed foam cube grasp-lift 的 clean success 数据，并建立完整标注和复核流程。

## Day 1: Hardware Sanity

- [ ] 运行 JAKA 连接检查。
- [ ] 运行 RH56 open/close 空载 10 次。
- [ ] 记录每根手指 command/state 延迟。
- [ ] 确认 `open`、`power_grasp`、`envelope_close` 三个 primitive。
- [ ] 拍摄每个 primitive 的侧视视频。
- [ ] 如果任何手指方向/幅度异常，先修标定，不进入物体测试。

## Day 2: Fixture and Object

- [ ] 准备 50-60 mm 泡沫块，质量 <30 g。
- [ ] 在桌面贴固定起始框。
- [ ] 定义 pregrasp pose。
- [ ] 固定 wrist orientation。
- [ ] close-only 10 trials。
- [ ] 输出 failure count：`object_pushed_before_close`、`grasp_empty`、`insufficient_closure`。

通过条件：close-only >=7/10。

## Day 3: Lift 3 cm

- [ ] close 后等待 0.8-1.0 s。
- [ ] lift 3 cm，hold 2 s。
- [ ] 采 10 trials。
- [ ] 每条写 `manual_review.yaml`。
- [ ] 如果 success <7/10，回到 Day 2 调 pregrasp/close_strength。

通过条件：lift 3 cm >=7/10。

## Day 4: Lift 8 cm

- [ ] lift height 改为 8 cm。
- [ ] 采 20 trials。
- [ ] 统计 success rate、slip count、mean final height。
- [ ] 把 clean success、weak success、failure 分开。

通过条件：lift 8 cm >=14/20。

## Day 5: First Dataset Batch

- [ ] 采 fixed foam cube lift 8 cm，目标 30-50 条 clean success。
- [ ] 保存视频和 steps。
- [ ] 抽样 replay 10 条 clean success。
- [ ] replay 失败的标 `use_for_bc=false`。

## Day 6: Minimal BC Smoke Test

- [ ] 只使用 clean success。
- [ ] 输入：`robot_q_current`、`ee_pose`、`hand_state`、stage one-hot。
- [ ] 输出：`ee_delta_xyz_m`、`grasp_type`、`close_strength`。
- [ ] 固定起点 rollout 10 次。
- [ ] 记录 BC success，而不是只看 train loss。

## Day 7: Review and Decision

- [ ] 汇总 Gate 表。
- [ ] 画 failure_mode bar chart。
- [ ] 决定下周是否加入 cylinder。
- [ ] 决定是否开始 pick-and-place。

## Week 1 Exit Criteria

继续到 Week 2 的条件：

- RH56 空载 open/close 10/10 正常。
- close-only >=70%。
- lift 8 cm >=70%。
- 至少 30 条 clean success。
- 每条都有 metadata、steps、video、manual_review。

如果未达标：

- 不训练 Diffusion Policy。
- 不加视觉闭环。
- 不换复杂物体。
- 回退到 close-only 或 lift 3 cm。
