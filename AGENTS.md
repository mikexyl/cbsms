# CBSMS Workspace Agent Instructions

This file is the authoritative agent guide for the whole CBSMS ROS 1 catkin
workspace. The workspace-root `AGENTS.md` should remain a short pointer to this
file.

## Project Structure

Source packages live under `src/`. Generated spaces live at the workspace root
in `build/`, `devel/`, `install/`, and `logs/`; do not delete, regenerate, or
commit them unless explicitly asked.

Core local packages:

- `src/cbs`: CBS/BPSAM factor graph and belief exchange logic.
- `src/liorf`: LiORF mapping, CBS bridge integration, and LiORF Rerun logging.
- `src/Kimera-VIO`: Kimera VIO core.
- `src/Kimera-VIO-ROS`: Kimera ROS launch and CBS bridge integration.
- `src/cbsms`: metapackage, Docker tooling, manifests, and workspace guidance.
- `src/aria_common`, `src/aria_viz`: Aria support and visualization packages.

Tests and examples are package-local, usually in `test/`, `tests/`, or
`examples/`. Runtime assets live in package-specific `launch/`, `params/`,
`rviz/`, `data/`, and `docs/` directories.

## Fresh Setup

A fresh agent should be able to clone only `src/cbsms`, import the pinned
repositories, build inside Docker, and run the S3E Alpha Kimera/LiORF smoke
test.

```bash
mkdir -p ~/workspaces/cbsms_ws/src
cd ~/workspaces/cbsms_ws
git clone git@github.com:mikexyl/cbsms.git src/cbsms
docker compose -f src/cbsms/docker/compose.yml build
src/cbsms/docker/start_container.sh
src/cbsms/docker/build_in_container.sh
```

`docker/build_workspace.sh` imports first-party repos from
`src/cbsms/cbsms.repos` and dependencies from
`src/cbsms/docker/cbsms.dependencies.repos`. If `/workspace/cbsms.repos` exists,
it intentionally overrides the package-local first-party manifest.

## Docker And Build Rules

Build and run ROS code inside Docker. The persistent container bind-mounts the
whole workspace at `/workspace`, so generated `build/`, `devel/`, and `logs/`
folders remain on the host.

Useful commands:

- `docker compose -f src/cbsms/docker/compose.yml build`: build the reusable ROS/CUDA image.
- `src/cbsms/docker/start_container.sh`: start the persistent `cbsms_ws` container.
- `src/cbsms/docker/build_in_container.sh`: import missing repos and build `cbsms`.
- `src/cbsms/docker/shell.sh`: open an interactive shell in the same container.
- `source devel/setup.bash`: load built workspace packages after a successful build.

Keep `--merge-devel`, Release mode, C++17, GTSAM unstable symbols, Expmap
retractions, system Eigen/Metis, TBB enabled, tangent preintegration disabled,
and tests/examples disabled unless explicitly tuning build behavior.

Focused CBS build:

```bash
docker compose -f src/cbsms/docker/compose.yml exec cbsms bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace; source devel/setup.bash; \
   catkin build cbs liorf kimera_vio kimera_vio_ros --no-status --summarize -j12 -p1'
```

## Runtime Checks

The S3E dataset is expected at `/data/s3e`, mounted read-only by compose. Runtime
tests should usually run for 60 seconds unless the user asks otherwise:

```bash
docker compose -f src/cbsms/docker/compose.yml exec cbsms bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace; source devel/setup.bash; \
   timeout 75s roslaunch kimera_vio_ros s3e_alpha_liorf_kimera_experiment.launch \
   bag_duration:=60.0 enable_cbs_bridge:=true rerun_visualizer_enable:=true'
```

Rerun streams to the Docker host at `rerun+http://172.18.0.1:9876/proxy` by
default. Keep Rerun enabled for both Kimera and LiORF during CBS debugging.
Actual RViz process startup is not required for the Rerun checks unless the user
explicitly asks to open RViz. Use `enable_cbs_bridge:=false` for the isolated
baseline.

Latest known 60 second CBS run from 2026-04-30:

- Kimera and LiORF both streamed visualization data to Rerun.
- The run finished without `Requested to eliminate a key that is not in the factors`.
- LiORF BPSAM root size stayed bounded in the observed logs.
- Outstanding debugging remains around LiORF accepting too few Kimera beliefs.

## Coding Style

Most code is C++17 with CMake/catkin. Match the surrounding package style:
two-space indentation in Kimera/CBS C++ and CMake, descriptive CamelCase types,
lower_snake_case variables where already used, and ROS package names in
lowercase with underscores.

Prefer existing package helpers and conventions over new abstractions. Shared
CBS/GTSAM compatibility helpers belong under `src/cbs/include/cbs/utils/`.
Keep changes narrowly scoped to the package and behavior under discussion.

## Testing

Use package-local tests where present: `src/Kimera-VIO/tests`,
`src/Kimera-VIO-ROS/test`, `src/Kimera-RPGO/tests`, and `src/aria_viz/tests`.
The default Docker build disables tests for speed with `-DBUILD_TESTING=OFF`;
enable tests explicitly when changing tested behavior, then run
`catkin run_tests` or build the affected package test target inside the
container.

For CBS runtime changes, verify at least:

- A focused build of `cbs`, `liorf`, `kimera_vio`, and `kimera_vio_ros`.
- A CBS-enabled 60 second S3E run.
- A CBS-disabled 60 second comparison when changing launch defaults,
  optimization timing, or belief exchange behavior.

## Git Hygiene

This workspace contains nested Git repositories. Commit changes in the package
repo that owns the files, not at the workspace root. Preserve unrelated local
changes and report any dirty package state before pushing.

Before handoff, run:

```bash
for r in src/*/.git; do d=${r%/.git}; git -C "$d" status --short | sed "s|^|$d |"; done
```

If a package has local edits that should be preserved, commit and push that
package, then update the relevant `.repos` SHA in `src/cbsms`.

## Pinned Package Revisions

Use the exact commit SHAs in `cbsms.repos` and
`docker/cbsms.dependencies.repos`. Branch names are orientation only:
`cbs` is from `prior-belief-factors`, `liorf`, `Kimera-VIO`, and
`Kimera-VIO-ROS` are from `dev/cbsms`, `aria_common` and `aria_viz` are from
`kimera-ros2`, and `gtsam` is a `mikexyl/gtsam` CBSMS branch based on the 4.1.1
checkout.

These manifests reproduce committed HEADs only. Check for dirty package state
before claiming a workspace is exactly reproducible.

## CBS Follow-Up Backlog For 2026-05-01

These are the active problems to pick up next. Do not treat them as solved just
because the latest 60 second run completed.

- LiORF still appears to accept only a small number of Kimera beliefs per CBS
  iteration. Inspect the detailed rejection counters first, especially
  `rejected_first_message`, timestamp mismatch, missing target key, and
  root/window gating.
- With `cbsEnableSoftReset` enabled, only the first belief for a new/reset peer
  should be rejected. Repeated first-message rejections likely indicate a reset
  lifecycle or sender/key bookkeeping problem.
- Confirm the exact LiORF acceptance gate for inbound Kimera beliefs. Because
  LiORF uses ISAM2 rather than IFLS, verify whether the target key must be in
  the current Bayes tree root and whether root-size limiting can starve otherwise
  valid Kimera beliefs.
- Keep monitoring LiORF root size and active CBS prior count. Beliefs must not
  indefinitely expand the active variable set.
- Re-check timestamp matching on the software-synced S3E dataset.
  `cbsBeliefTimestampToleranceSec` is in seconds, so `0.12` means 120 ms, not
  0.12 ms. If many software-synced beliefs miss the tolerance, inspect timestamp
  source conversion before increasing the threshold.
- Clarify `added_to_factor_graph_per_update` semantics in logs: it should be
  obvious whether the counter includes only brand-new belief keys or also
  replacement updates for already tracked CBS priors.
- Tune LiORF outgoing belief covariance scale much smaller than Kimera's. Prefer
  making the LiORF-to-Kimera covariance scale configurable, with a conservative
  small default, then use the covariance audit logs to compare against Kimera's
  outgoing belief scale.
- Keep Rerun visualization enabled for both Kimera and LiORF in the experiment
  launch while debugging. RViz can remain launch-configurable, but the Rerun
  signal is the required visualization path for these checks.
