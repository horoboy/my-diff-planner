# FUEL and Diff-Planner integration workspace

This branch keeps the FUEL, Diff-Planner, and NLopt repositories separate while
tracking the development-container, SITL validation, and VS Code integration
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
├── src/FUEL/       # Customized FUEL history (fuel-integration branch)
├── Diff-Planner/   # Customized Diff-Planner history (main branch)
├── nlopt/          # NLopt upstream repository
├── px4ctrl_sitl_ws/src/fuel_px4ctrl_sitl/
├── .devcontainer/
└── .vscode/
```

Catkin build outputs (`build/`, `devel/`, and `install/`) are intentionally not
tracked.

## Submodule branches

The customized FUEL history is published as `fuel-integration` in this
repository. Diff-Planner remains on `main`. The integration branch records exact
submodule commits, so normal builds should use `git submodule update --init
--recursive` instead of applying the historical patches under `patches/`.

PX4-Autopilot itself and generated SITL output are intentionally excluded
because they are large third-party/runtime artifacts.
