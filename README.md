# CBSMS Workspace Metapackage

`cbsms` is the setup anchor for the CBS multi-sensor mapping ROS 1 workspace.
Clone this package first; it then imports and builds Kimera-VIO, LiORF, CBS,
Aria visualization, GTSAM 4.1.1, and the local Rerun SDK wrapper in a persistent
CUDA/ROS Noetic Docker container.

## Prerequisites

- Docker with Compose support and access to the base image
  `cuda-ros-base:noetic`.
- SSH agent access to the private Git remotes in `cbsms.repos`.
- Optional runtime data at `/data/s3e/S3E_Square_1.bag`.
- Optional Rerun viewer on the host listening at `127.0.0.1:9876`.

## Fresh Workspace Setup

Start from an empty catkin workspace root:

```bash
mkdir -p ~/workspaces/cbsms_ws/src
cd ~/workspaces/cbsms_ws
git clone git@github.com:mikexyl/cbsms.git src/cbsms
docker compose -f src/cbsms/docker/compose.yml build
src/cbsms/docker/start_container.sh
src/cbsms/docker/build_in_container.sh
```

The build script imports missing repositories from `src/cbsms/cbsms.repos` and
`src/cbsms/docker/cbsms.dependencies.repos`, then runs `catkin build` with
Kimera/GTSAM-compatible flags. A root-level `cbsms.repos` overrides the
package-local manifest if present.

## Workspace Layout

- `src/cbsms/cbsms.repos`: first-party package SHAs.
- `src/cbsms/docker/cbsms.dependencies.repos`: dependency package SHAs.
- `src/cbsms/docker/compose.yml`: persistent `cbsms_ws` container.
- `src/cbsms/docker/build_workspace.sh`: import/build implementation.
- `src/{cbs,liorf,Kimera-VIO,Kimera-VIO-ROS}`: core packages.
- `build/`, `devel/`, `install/`, `logs/`: generated host-side catkin folders.

## Docker Workflow

Use Docker for all builds and ROS runs:

```bash
src/cbsms/docker/shell.sh
source /opt/ros/noetic/setup.bash
cd /workspace
source devel/setup.bash
```

The Dockerfile installs system packages only. Source packages are built in the
bind-mounted workspace so incremental `build/`, `devel/`, and `logs/` outputs
persist outside the container. The compose file also mounts `/data/s3e` read-only
and maps `host.docker.internal` to the Docker host.

## S3E Alpha Smoke Test

Run the combined Kimera/LiORF launch for the standard 60-second test window:

```bash
docker compose -f src/cbsms/docker/compose.yml exec cbsms bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace; source devel/setup.bash; \
   timeout 75s roslaunch kimera_vio_ros s3e_alpha_liorf_kimera_experiment.launch \
   bag_duration:=60.0 enable_cbs_bridge:=true rerun_visualizer_enable:=true'
```

Use `enable_cbs_bridge:=false` for an isolated baseline. Rerun streams to
`rerun+http://172.18.0.1:9876/proxy` by default so the host viewer can see data
from inside Docker.

## Experiment Reports

For repeatable CBS runs with trajectory and belief-exchange metrics, use the
host-side report runner:

```bash
src/cbsms/tools/cbsms_experiment.py run --name s3e-square1-cbs --duration 60
```

It writes a run folder under `runs/` with `summary.md`, `summary.json`, raw
odometry CSVs, parsed CBS transport/roundtrip/merge CSVs, covariance summaries,
40-sample injected-belief covariance tables, TUM trajectories, evo APE/RPE
metrics when `evo` is installed, and RMSE-style trajectory metrics. See
[`docs/experiment_reports.md`](docs/experiment_reports.md) for the full
workflow and common variants.

For the current working understanding of the same-drone Kimera/LiORF CBS setup,
factor semantics, belief windows, and report interpretation, see
[`docs/project_understanding.md`](docs/project_understanding.md).

## Reproducibility

The `.repos` files pin exact commit SHAs matching this workspace's committed
HEADs. They do not include uncommitted edits. Before handing this off, commit
and push dirty package changes, then update the manifests if any HEAD changed.
