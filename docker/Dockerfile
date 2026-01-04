# docker/Dockerfile
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ---------- Core system + ROS tooling ----------
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    vim \
    gdb \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    i2c-tools \
    libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------- rosdep (safe on PC & Pi) ----------
RUN rosdep init || true
RUN rosdep update

# ---------- Workspace ----------
WORKDIR /ros2_ws

# ---------- Entrypoint ----------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
