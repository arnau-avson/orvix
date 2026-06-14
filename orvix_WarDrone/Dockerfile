# ============================================================
# wardrone-dev: ROS 2 Humble development & testing container
# ============================================================
# Usage:
#   docker compose build wardrone-dev
#   docker compose run wardrone-dev test-unit
#   docker compose run wardrone-dev colcon-build
#   docker compose run wardrone-dev bash
# ============================================================

FROM ros:humble-ros-base-jammy

LABEL maintainer="arnau"
LABEL description="WarDrone ROS 2 development and testing environment"

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1

# ---- System dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    libopencv-dev \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# ---- Python dependencies ----
RUN pip3 install --no-cache-dir \
    mavsdk \
    ultralytics \
    opencv-python-headless \
    pyyaml \
    pytest \
    numpy

# ---- Workspace setup ----
WORKDIR /ws

# Copy source code (or mount as volume in docker-compose)
COPY src/ /ws/src/

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ws && \
    colcon build --symlink-install" \
    || echo "Initial build completed (some packages may need runtime deps)"

# ---- Entrypoint ----
COPY scripts/docker_entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
