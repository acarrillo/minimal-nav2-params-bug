FROM ros:jazzy-ros-base

# Install Nav2 packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-msgs \
    ros-jazzy-map-msgs \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Setup entrypoint
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /workspace

CMD ["/bin/bash"]