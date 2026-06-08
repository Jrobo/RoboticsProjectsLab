FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=burger

# Install ROS2, TurtleBot3, Nav2, Gazebo, and development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-rviz2 \
    gazebo \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/tbot3_nav_monitor_ws/src

# Copy package into container
COPY . /root/tbot3_nav_monitor_ws/src/tbot3_nav_monitor

# Build package
WORKDIR /root/tbot3_nav_monitor_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source ROS and workspace automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/tbot3_nav_monitor_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc

CMD ["/bin/bash"]
