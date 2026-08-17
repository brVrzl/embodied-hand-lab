# Third-party source policy

## English

Current training/runtime upstreams are Git submodules:

- `third_party/openpi`: pinned OpenPI source for the π0.5 adapter;
- `third_party/lerobot`: pinned LeRobot source for ACT/training views.

Initialize them with:

```bash
git submodule update --init --recursive
scripts/check_training_dependencies.sh --help
```

Do not advance a submodule independently. Vendor snapshots under this directory
are attribution-bound reference material; current project integrations belong in
`src/`, `tools/`, and `training/`.

## 中文

当前 training/runtime upstream 是 Git submodule：

- `third_party/openpi`：π0.5 adapter 使用的固定 OpenPI source；
- `third_party/lerobot`：ACT/training view 使用的固定 LeRobot source。

使用以下命令初始化：

```bash
git submodule update --init --recursive
scripts/check_training_dependencies.sh --help
```

不要独立推进 submodule。本目录下的 vendor snapshot 是需要 attribution 的 reference material；当前项目集成代码
应位于 `src/`、`tools/` 和 `training/`。
