# Third-party source policy

Git repositories that are part of a maintained training/runtime workflow live
here as Git submodules with the exact commit recorded by the parent repository:

- `openpi`: `Physical-Intelligence/openpi`, used by the project π0.5 adapter;
- `lerobot`: `huggingface/lerobot`, used by the project ACT/LeRobot launcher.

Clone this repository with submodules:

```bash
git clone --recurse-submodules <embodied-lab-repository-url>
```

For an existing clone, initialize or repair them with:

```bash
git submodule update --init --recursive
scripts/check_training_dependencies.sh --require-image
```

Do not advance a submodule independently. Check out the desired upstream
commit inside it, then commit the parent repository's gitlink and `.gitmodules`
change together. The project launchers mount these sources read-only and verify
their pinned commits before training.

The `inspire_hand` and `jaka_sdk` directories are supplied vendor snapshots,
not runtime Git repositories with a recoverable upstream commit in this
worktree. They remain ordinary tracked assets. Python packages, Docker images,
model weights, caches, datasets, logs, and checkpoints are runtime artifacts;
they are pinned through project manifests or image digests and are not copied
into this directory or committed to Git.
