# Repository Guidelines

## Agent Objective

This package is the bootstrap source for the CBSMS ROS 1 workspace. A fresh
agent should be able to clone only `src/cbsms`, import the pinned repositories,
build inside Docker, and run the S3E Alpha Kimera/LiORF smoke test.

## Fresh Setup Checklist

1. Confirm Docker Compose works and `cuda-ros-base:noetic` is available.
2. Confirm the host SSH agent can access the private GitHub remotes.
3. Create a catkin workspace and clone this package:

```bash
mkdir -p ~/workspaces/cbsms_ws/src
cd ~/workspaces/cbsms_ws
git clone git@github.com:mikexyl/cbsms.git src/cbsms
```

4. Build and start the persistent container:

```bash
docker compose -f src/cbsms/docker/compose.yml build
src/cbsms/docker/start_container.sh
src/cbsms/docker/build_in_container.sh
```

`docker/build_workspace.sh` imports first-party repos from `src/cbsms/cbsms.repos`
and dependencies from `src/cbsms/docker/cbsms.dependencies.repos`. If
`/workspace/cbsms.repos` exists, it intentionally overrides the package-local
first-party manifest.

## Docker Rules

Build and run ROS code inside Docker only. The container is persistent and bind
mounts the whole workspace at `/workspace`, so generated `build/`, `devel/`, and
`logs/` folders remain on the host. Do not compile source packages in the
Dockerfile and do not delete generated folders unless explicitly asked.

The catkin build must keep `--merge-devel`, Release mode, C++17, GTSAM unstable
symbols, Expmap retractions, system Eigen/Metis, TBB enabled, tangent
preintegration disabled, and tests/examples disabled unless explicitly tuning
build behavior.

## Runtime Checks

The S3E dataset is expected at `/data/s3e`, mounted read-only by compose. Runtime
tests should run for 60 seconds unless the user asks otherwise:

```bash
docker compose -f src/cbsms/docker/compose.yml exec cbsms bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace; source devel/setup.bash; \
   timeout 75s roslaunch kimera_vio_ros s3e_alpha_liorf_kimera_experiment.launch \
   bag_duration:=60.0 enable_cbs_bridge:=true rerun_visualizer_enable:=true'
```

Rerun streams to the Docker host at `rerun+http://172.18.0.1:9876/proxy` by
default. Use `enable_cbs_bridge:=false` to disable Kimera/LiORF belief sharing.

## Pinned Package Revisions

Use the exact commit SHAs in `cbsms.repos` and
`docker/cbsms.dependencies.repos`. Branch names are only orientation:
`cbs` is from `fresh-rebuild`, `liorf`, `Kimera-VIO`, and `Kimera-VIO-ROS` are
from `dev/cbsms`, `aria_common` and `aria_viz` are from `kimera-ros2`, and
`gtsam` is the 4.1.1 checkout.

These manifests reproduce committed HEADs only. Check for dirty package state
before claiming a workspace is exactly reproducible.

## Git Hygiene

This workspace contains nested Git repositories. Commit changes in the package
repo that owns the files, not at the workspace root. Preserve unrelated local
changes and report any dirty package state before pushing.

Before handoff, run:

```bash
for r in src/*/.git; do d=${r%/.git}; git -C "$d" status --short | sed "s|^|$d |"; done
```

If a package has local edits, commit and push that package, then update the
relevant `.repos` SHA.
