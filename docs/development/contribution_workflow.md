# Contribution workflow

1. Inspect root, branch, HEAD, upstream, linked worktrees, and dirty/untracked
   files.
2. Read `AGENTS.md`, the current status, and the authoritative topic page.
3. Preserve concurrent/user-owned work and use the current worktree only.
4. Make the smallest behaviorally justified change.
5. Use temporary validation as needed, then remove it unless it protects a
   stable contract, safety behavior, real regression, or key workflow. Prefer
   extending an existing behavior-level test over adding a new fine-grained
   file.
6. Run focused checks, then the full applicable offline suite.
7. Inspect staged diff and `git diff --check`; commit a coherent scope.
8. Fetch, inspect ahead/behind, integrate remote changes without history
   rewriting, push the current feature branch, and verify its remote commit.

Never force-push, squash existing history, use `git clean`, reset user changes,
or modify another linked worktree. Do not include local captures, calibration
assets, secrets, or unrelated experiments.

A physical result requires its own authorized session and evidence. Repository
maintenance cannot declare a pending hardware correction physically passed.

---

# 中文版：贡献流程

1. 检查仓库根目录、分支、HEAD、上游、关联 worktree 以及已修改/未跟踪文件。
2. 阅读 `AGENTS.md`、当前状态页和该主题的权威页面。
3. 保护并发或用户自有工作，只使用当前 worktree。
4. 只做能够从行为证据证明必要的最小修改。
5. 按需使用临时验证；除非它保护稳定契约、安全行为、真实回归或关键流程，否则任务结束前
   删除。优先扩展已有行为级测试，不默认新增细粒度测试文件。
6. 先运行聚焦检查，再运行完整适用的离线测试集。
7. 检查暂存差异和 `git diff --check`，提交一个边界清晰的改动。
8. fetch 后检查 ahead/behind；不改写历史地整合远端改动，推送当前分支并核对远端提交。

绝不能 force-push、压缩已有历史、使用 `git clean`、重置用户改动或修改其他关联
worktree。不要提交本地采集、标定资产、秘密信息或无关实验。

真机结果需要独立授权的会话和证据。仓库维护不能把尚待验证的硬件修正宣称为真机通过。
