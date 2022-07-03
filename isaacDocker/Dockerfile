# Jetpack 4.6.1
# Docker file for aarch64 based Jetson device
FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1
# L4T variables
ENV L4T=32.7
ENV L4T_MINOR_VERSION=7.1
# Configuration CUDA
ENV CUDA=10.2

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# GStreamer
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev && \
    rm -rf /var/lib/apt/lists/*

# Install OpenCV dependencies
RUN apt-get update && apt-get install -y \
    libavformat-dev \
    libjpeg-dev \
    libopenjp2-7-dev \
    libpng-dev \
    libpq-dev \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libtiff-dev \
    pkg-config \
    yasm && \
    rm -rf /var/lib/apt/lists/*

# Install additional packages needed for ROS2 dependencies
RUN apt-get update && apt-get install -y \
    python3-distutils \
    libboost-all-dev \
    libboost-dev \
    libpcl-dev \
    libode-dev \
    lcov \
    python3-zmq \
    libxaw7-dev \
    libgraphicsmagick++1-dev \
    graphicsmagick-libmagick-dev-compat \
    libceres-dev \
    libsuitesparse-dev \
    libncurses5-dev \
    libassimp-dev \
    libyaml-cpp-dev \
    libpcap-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Git-LFS and other packages
RUN apt-get update && apt-get install -y \
    git-lfs \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Fix cuda info
ARG DPKG_STATUS
# Add nvidia repo/public key and install VPI libraries
RUN echo "$DPKG_STATUS" >> /var/lib/dpkg/status && \
    curl https://repo.download.nvidia.com/jetson/jetson-ota-public.asc > /etc/apt/trusted.gpg.d/jetson-ota-public.asc && \
    echo "deb https://repo.download.nvidia.com/jetson/common r${L4T} main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    apt-get update && apt-get install -y libnvvpi1 vpi1-dev && \
    rm -rf /var/lib/apt/lists/*

# Update environment
ENV LD_LIBRARY_PATH="/opt/nvidia/vpi1/lib64:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda-${CUDA}/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}"
ENV LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}"

# Install pcl_conversions & sensor_msgs_py
ARG ROSINSTALL=isaac_ros_base.rosinstall
# Copy wstool rosinstall
COPY ${ROSINSTALL} ${ROSINSTALL}

# Fix image transport to v3.0.0
# https://github.com/stereolabs/zed-ros2-wrapper/issues/66
RUN apt-get update && \
    apt-get install python3-vcstool python3-pip -y && \
    mkdir -p ${ROS_ROOT}/src && \
    vcs import ${ROS_ROOT}/src < ${ROSINSTALL} && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    cd ${ROS_ROOT} && \
    rosdep install -y --ignore-src --from-paths src --rosdistro foxy && \
    colcon build --merge-install --packages-up-to pcl_conversions sensor_msgs_py diagnostic_updater xacro \
    camera_calibration_parsers image_transport image_common camera_info_manager ros2_shared gscam2 && \
    rm -Rf src logs build

# Install gcc8 for cross-compiled binaries from Ubuntu 20.04
RUN apt-get update && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    apt-get install -y gcc-8 g++-8 libstdc++6 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8 && \
    rm -rf /usr/bin/aarch64-linux-gnu-gcc /usr/bin/aarch64-linux-gnu-g++ \
        /usr/bin/aarch64-linux-gnu-g++-7 /usr/bin/aarch64-linux-gnu-gcc-7 && \
    update-alternatives --install /usr/bin/aarch64-linux-gnu-gcc aarch64-linux-gnu-gcc \
        /usr/bin/gcc-8 8 && \
    update-alternatives --install /usr/bin/aarch64-linux-gnu-g++ aarch64-linux-gnu-g++ \
        /usr/bin/g++-8 8 && \
    rm -rf /var/lib/apt/lists/*

################ BUILD & INSTALL ISAAC ROS packages ####################

# Build Isaac ROS package
ENV ISAAC_ROS_WS /opt/isaac_ros_ws
ARG ROSINSTALL=isaac.rosinstall
# Copy wstool rosinstall
COPY ${ROSINSTALL} ${ROSINSTALL}

RUN apt-get update && \
    apt-get install python3-vcstool python3-pip -y && \
    mkdir -p ${ISAAC_ROS_WS}/src && \
    vcs import ${ISAAC_ROS_WS}/src < ${ROSINSTALL} && \
    rm -rf /var/lib/apt/lists/*
# Pull LFS files
RUN cd ${ISAAC_ROS_WS}/src/isaac_ros_common && git lfs pull && \
    cd ${ISAAC_ROS_WS}/src/isaac_ros_image_pipeline && git lfs pull && \
    cd ${ISAAC_ROS_WS}/src/isaac_ros_argus_camera && git lfs pull

# Change workdir
WORKDIR $ISAAC_ROS_WS

# Build Isaac ROS
RUN . /opt/ros/$ROS_DISTRO/install/setup.sh && \
    colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

################ Final enviroment setup ####################

# Use CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_ROOT/install/setup.bash"' \
      /ros_entrypoint.sh

RUN sed --in-place --expression \
      '$isource "$ISAAC_ROS_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run ros package launch file
#CMD ["ros2", "launch", "isaac_ros_argus_camera_mono", "isaac_ros_argus_camera_mono_launch.py"]
