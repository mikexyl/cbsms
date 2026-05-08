FROM cuda-ros-base:noetic

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      apt-utils \
      bash-completion \
      build-essential \
      ca-certificates \
      ccache \
      curl \
      gfortran \
      git \
      gnupg \
      libatlas-base-dev \
      libboost-all-dev \
      libeigen3-dev \
      libgeographic-dev \
      libgflags-dev \
      libgl1-mesa-dev \
      libglew-dev \
      libgoogle-glog-dev \
      libgraphviz-dev \
      libgsl-dev \
      libgtk-3-dev \
      libjpeg-dev \
      libmetis-dev \
      libopencv-dev \
      libparmetis-dev \
      libpng-dev \
      libpopt-dev \
      libspdlog-dev \
      libsuitesparse-dev \
      libtbb-dev \
      libtiff-dev \
      libunwind-dev \
      libvtk7-dev \
      libyaml-cpp-dev \
      ninja-build \
      openssh-client \
      pkg-config \
      python3-catkin-tools \
      python3-pip \
      python3-rosdep \
      python3-vcstool \
      python3-wstool \
      unzip \
      wget \
      ros-noetic-cv-bridge \
      ros-noetic-geographic-msgs \
      ros-noetic-geometry-msgs \
      ros-noetic-image-geometry \
      ros-noetic-image-pipeline \
      ros-noetic-image-transport \
      ros-noetic-message-filters \
      ros-noetic-nav-msgs \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      ros-noetic-rosbag \
      ros-noetic-rviz \
      ros-noetic-sensor-msgs \
      ros-noetic-std-msgs \
      ros-noetic-tf \
      ros-noetic-visualization-msgs && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir "cmake>=3.26,<3.29"

ARG EIGEN_VERSION=3.4.0
ARG ONETBB_VERSION=2021.13.0

RUN set -euxo pipefail; \
    tmp_dir="$(mktemp -d)"; \
    curl -fsSL \
      "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz" \
      -o "${tmp_dir}/eigen.tar.gz"; \
    tar -xzf "${tmp_dir}/eigen.tar.gz" -C "${tmp_dir}"; \
    cmake -S "${tmp_dir}/eigen-${EIGEN_VERSION}" \
      -B "${tmp_dir}/eigen-build" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DEIGEN_BUILD_DOC=OFF \
      -DBUILD_TESTING=OFF; \
    cmake --build "${tmp_dir}/eigen-build" --target install; \
    curl -fsSL \
      "https://github.com/oneapi-src/oneTBB/archive/refs/tags/v${ONETBB_VERSION}.tar.gz" \
      -o "${tmp_dir}/onetbb.tar.gz"; \
    tar -xzf "${tmp_dir}/onetbb.tar.gz" -C "${tmp_dir}"; \
    cmake -S "${tmp_dir}/oneTBB-${ONETBB_VERSION}" \
      -B "${tmp_dir}/onetbb-build" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DTBB_TEST=OFF \
      -DTBB_STRICT=OFF; \
    cmake --build "${tmp_dir}/onetbb-build" --target install -j"$(nproc)"; \
    ldconfig; \
    rm -rf "${tmp_dir}"

ARG CBSMS_UID=1000
ARG CBSMS_GID=1000

RUN if ! getent group "${CBSMS_GID}" >/dev/null; then \
      groupadd --gid "${CBSMS_GID}" cbsms; \
    fi && \
    if ! getent passwd "${CBSMS_UID}" >/dev/null; then \
      useradd --uid "${CBSMS_UID}" --gid "${CBSMS_GID}" --create-home --shell /bin/bash cbsms; \
    fi && \
    mkdir -p /home/cbsms/.ssh /workspace && \
    chown -R "${CBSMS_UID}:${CBSMS_GID}" /home/cbsms /workspace

ENV CMAKE_PREFIX_PATH=/usr/local:/opt/ros/noetic
ENV Eigen3_DIR=/usr/local/share/eigen3/cmake
ENV TBB_DIR=/usr/local/lib/cmake/TBB
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib64:${LD_LIBRARY_PATH}
ENV CCACHE_DIR=/workspace/.ccache

WORKDIR /workspace
CMD ["bash"]
