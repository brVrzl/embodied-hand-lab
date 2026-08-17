# Runtime log schemas

## English

Runtime JSON/JSONL records are owned by the subsystem that writes them. Read
the schema/version field before consuming a record. The important current
families are:

- Quest/UMIP recording: `.umip.jsonl`, with header/sample/footer records;
- episode metadata, canonical samples, quality, and timing under episode roots;
- native JAKA worker metrics and event JSONL;
- RH56 telemetry and command diagnostics;
- ACT/π0.5 reports and status under ignored `outputs/`.

These are runtime interfaces, not committed experiment results. Do not add a
new duplicate manifest or checksum file just to inspect an existing record.

## 中文

运行时 JSON/JSONL record 由写入它的 subsystem 负责。消费 record 前先读取 schema/version 字段。当前重要
record family 包括：

- Quest/UMIP recording：`.umip.jsonl`，包含 header/sample/footer；
- episode root 下的 metadata、canonical sample、quality 和 timing；
- native JAKA worker metrics 和 event JSONL；
- RH56 telemetry 和 command diagnostics；
- ignored `outputs/` 下的 ACT/π0.5 report 和 status。

这些是运行时接口，不是提交到仓库的实验结果。不要为了检查现有 record 而创建重复的 manifest 或 checksum 文件。
