#!/usr/bin/env bash
set -eo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source /opt/ros/noetic/setup.bash
set -u

workspace="${CBSMS_WORKSPACE:-/workspace}"
cd "${workspace}"

git config --global --add safe.directory '*'

export CMAKE_PREFIX_PATH="/usr/local:/opt/ros/noetic:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
export GIT_SSH_COMMAND="${GIT_SSH_COMMAND:-ssh -o StrictHostKeyChecking=accept-new}"

cmake_args=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_CXX_STANDARD=17
  -DBUILD_TESTING=OFF
  -DKIMERA_BUILD_TESTS=OFF
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
  -DGTSAM_BUILD_TESTS=OFF
  -DGTSAM_BUILD_UNSTABLE=ON
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
  -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF
  -DGTSAM_POSE3_EXPMAP=ON
  -DGTSAM_ROT3_EXPMAP=ON
  -DGTSAM_TANGENT_PREINTEGRATION=OFF
  -DGTSAM_USE_SYSTEM_EIGEN=ON
  -DGTSAM_USE_SYSTEM_METIS=ON
  -DGTSAM_WITH_TBB=ON
)

if [[ "${CBSMS_IMPORT_REPOS:-1}" == "1" ]]; then
  vcs import --skip-existing --input cbsms.repos .
  vcs import --skip-existing --input src/cbsms/docker/cbsms.dependencies.repos .
fi

catkin init
catkin config \
  --extend /opt/ros/noetic \
  --merge-devel \
  --no-install \
  --cmake-args "${cmake_args[@]}"

catkin build gtsam rerun_sdk --no-status --summarize "$@"
if [[ -f "${workspace}/devel/setup.bash" ]]; then
  set +u
  source "${workspace}/devel/setup.bash"
  set -u
else
  export CMAKE_PREFIX_PATH="${workspace}/devel:${CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${workspace}/devel/lib:${LD_LIBRARY_PATH}"
fi

catkin build cbsms --no-status --summarize "$@"
