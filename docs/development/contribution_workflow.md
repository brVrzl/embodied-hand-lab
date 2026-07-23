# Contribution workflow

1. Inspect root, branch, HEAD, upstream, linked worktrees, and dirty/untracked
   files.
2. Read `AGENTS.md`, the current status, and the authoritative topic page.
3. Preserve concurrent/user-owned work and use the current worktree only.
4. Make the smallest behaviorally justified change.
5. Add or update regression tests and current documentation together.
6. Run focused checks, then the full applicable offline suite.
7. Inspect staged diff and `git diff --check`; commit a coherent scope.
8. Fetch, inspect ahead/behind, integrate remote changes without history
   rewriting, push the current feature branch, and verify its remote commit.

Never force-push, squash existing history, use `git clean`, reset user changes,
or modify another linked worktree. Do not include local captures, calibration
assets, secrets, or unrelated experiments.

A physical result requires its own authorized session and evidence. Repository
maintenance cannot declare a pending hardware correction physically passed.
