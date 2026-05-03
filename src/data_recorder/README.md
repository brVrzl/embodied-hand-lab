# data_recorder

统一 episode recorder。

每个 episode 至少记录：

- timestamp
- task_name
- natural_language_instruction
- rgb/depth frame path
- arm joint states
- arm ee pose
- hand states
- dog states
- action
- success/failure
- operator notes

第一版落盘：

- 原始：`metadata.json + steps.jsonl + rgb/*.npy + depth/*.npy`
- 导出：结构化样本 JSONL + LeRobot 风格 stub

