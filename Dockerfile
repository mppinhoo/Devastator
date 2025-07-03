#Use the official ROS 2 Humble base image
FROM arm64v8/ros:humble-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV COLCON_WS=/proj_ws

# Install system dependencies and ROS 2 packages
RUN apt-get update && apt-get install -y \
    nano \
    locales \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    libbullet-dev \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    #ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-twist-mux \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    #ros-humble-gazebo-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-robot-state-publisher \
    ros-humble-laser-proc \
    jstest-gtk \
    usbutils \
    alsa-utils \
    pulseaudio \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace and copy your ROS2 workspace into the Docker image
RUN mkdir -p $COLCON_WS/src
WORKDIR $COLCON_WS
COPY src/ $COLCON_WS/src/

# # Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set locale
# RUN locale-gen en_US en_US.UTF-8 && \
#     update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
#     export LANG=en_US.UTF-8

# Initialize rosdep if not already initialized
# RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
#     rosdep init; \
#     fi && \
#     rosdep update


# Install dependencies for the workspace
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Source the ROS 2 setup script and build the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"

# Source the ROS 2 setup script
RUN echo "source $COLCON_WS/install/setup.bash" >> /root/.bashrc

# Set up entrypoint
CMD ["bash"]
