FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git vim wget curl nano \
    ca-certificates \
    build-essential cmake gdb \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-message-filters \
    ros-noetic-visualization-msgs \
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-graph \
    ros-noetic-gazebo-ros \
    libgoogle-glog-dev \
    libgflags-dev \
    ros-noetic-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && apt-get update \
    && apt-get install -y cuda-nvcc-11-8 cuda-cudart-dev-11-8 cuda-driver-dev-11-8 \
    && rm cuda-keyring_1.1-1_all.deb \
    && rm -rf /var/lib/apt/lists/*

ENV PATH=/usr/local/cuda-11.8/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH}

RUN rosdep init || true

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
