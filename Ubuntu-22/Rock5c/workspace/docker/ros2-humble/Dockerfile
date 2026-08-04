FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    nano \
    vim \
    less \
    tree \
    iproute2 \
    iputils-ping \
    net-tools \
    usbutils \
    pciutils \
    python3-pip \
    python3-serial \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /workspace/ros2_ws

CMD ["bash"]
