# FUEL and Diff-Planner integration workspace

This branch keeps the FUEL, Diff-Planner, and NLopt upstream repositories
separate while tracking the development-container and VS Code integration
configuration at the workspace root.

## Clone

```bash
git clone --branch fuel-diff-integration --recursive \
  https://github.com/horoboy/my-diff-planner.git
cd my-diff-planner
```

If the repository was cloned without `--recursive`, initialize all nested
submodules with:

```bash
git submodule update --init --recursive
```

## Workspace layout

```text
.
├── src/FUEL/       # FUEL upstream repository
├── Diff-Planner/   # Diff-Planner upstream repository
├── nlopt/          # NLopt upstream repository
├── .devcontainer/
└── .vscode/
```

Catkin build outputs (`build/`, `devel/`, and `install/`) are intentionally not
tracked.

## Preserved local FUEL change

The local CUDA configuration that existed when this integration branch was
created is stored in:

```text
patches/FUEL-enable-cuda-sm86.patch
```

Apply it after initializing submodules:

```bash
git -C src/FUEL apply ../../patches/FUEL-enable-cuda-sm86.patch
```
