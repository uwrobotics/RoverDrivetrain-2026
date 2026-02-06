# Dockerfile for Differential Drive Controller
# Build: docker build -t diff_drive_controller .
# Run:   docker run --rm -it --privileged --network=host diff_drive_controller

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    can-utils \
    iproute2 \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Copy ODrive packages
COPY odrive_base/ src/odrive_base/
COPY odrive_node/ src/odrive_node/
COPY odrive_ros2_control/ src/odrive_ros2_control/

# Copy diff_drive package
COPY diff_drive/ src/diff_drive/

# Install ROS dependencies
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Copy FastDDS config
COPY config/fastdds.xml /ros2_ws/fastdds.xml

# Setup entrypoint
COPY <<'EOF' /ros_entrypoint.sh
#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# Execute command
exec "$@"
EOF

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command: launch diff drive controller
CMD ["ros2", "launch", "diff_drive", "diff_drive.launch.py"]
