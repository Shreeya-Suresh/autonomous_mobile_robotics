FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# ---------- Antigravity dependencies (CRITICAL for port forwarding) ----------
RUN apt-get update && apt-get install -y \
    socat \
    netcat-openbsd \
    curl \
    wget \
    ca-certificates \
    bash \
    sudo \
    \
    # ---------- Minimal tools ----------
    build-essential \
    git \
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

# ---------- Create non-root user 'ros' ----------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ---------- rosdep init ----------
RUN rosdep init || true
RUN rosdep update

# ---------- Switch to non-root user ----------
USER $USERNAME

# ---------- Workspace ----------
WORKDIR /home/$USERNAME/ros2_ws

CMD ["bash"]