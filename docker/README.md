# CBSMS Docker

This image provides the OS and ROS tooling needed to build the CBSMS ROS 1
workspace on top of `cuda-ros-base:noetic`.

The Dockerfile does not compile source dependencies. Source dependencies are
workspace packages imported or provided under `src/`:

- `gtsam` from tag `4.1.1`
- `aria_common` from branch `kimera-ros2`
- `aria_viz` from branch `kimera-ros2`
- `rerun_sdk`, a local vendor package that downloads the Rerun C++ SDK during
  `catkin build`

The whole workspace is bind-mounted at `/workspace`. The persistent container
does not own the source tree; `catkin build --merge-devel` writes `build/`,
`devel/`, and `logs/` back into this host workspace.

By default the container runs as UID/GID `1000:1000` so generated files are
owned by the host user. Override `CBSMS_DOCKER_UID` and `CBSMS_DOCKER_GID` if
needed before building the image.

Build the image once, keep the container alive, and compile inside it:

```bash
docker compose -f src/cbsms/docker/compose.yml build
src/cbsms/docker/start_container.sh
src/cbsms/docker/build_in_container.sh
```

For an interactive shell in the same persistent container:

```bash
src/cbsms/docker/shell.sh
```

The build script imports missing repositories from `cbsms.repos` and
`src/cbsms/docker/cbsms.dependencies.repos`. The Aria dependencies use the
latest branch by commit date, `kimera-ros2`, from:

- `git@github.com:mikexyl/aria_common.git`
- `git@github.com:mikexyl/aria_visualization.git`

The catkin configuration follows Kimera's recommended `--merge-devel` workflow
and GTSAM flags: Release build, TBB, unstable symbols, full Expmap retractions,
system Eigen/Metis, no tangent preintegration, no GTSAM tests/examples, and no
`march=native`. LIORF does not publish extra catkin flags; its package CMake
sets Release, C++17, `-O3 -Wall -g -pthread` internally.
