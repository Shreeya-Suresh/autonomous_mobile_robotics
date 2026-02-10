# docker/Dockerfile
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ---------- Minimal tools ----------
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    vim \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools \
    \
    # ---- SLAM only ----
    ros-humble-slam-toolbox \
    \
    # ---- hardware utils ----
    i2c-tools \
    libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------- rosdep init (no install here) ----------
RUN rosdep init || true

# ---------- Workspace ----------
WORKDIR /ros2_ws

CMD ["bash"]
