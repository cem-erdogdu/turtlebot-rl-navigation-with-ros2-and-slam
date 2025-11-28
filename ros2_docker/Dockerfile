# Adapted from: https://github.com/Tiryoh/docker-ros2-desktop-vnc
# This Dockerfile builds an Ubuntu-based ROS 2 Galactic environment with LXDE desktop and VNC.
# It’s used by Unity Robotics’ Nav2 SLAM example to run ROS inside Docker.

FROM dorowu/ubuntu-desktop-lxde-vnc:focal
LABEL maintainer="Unity Robotics <unity-robotics@unity3d.com>"

# --------------------------------------------------------------------
# Environment configuration
# --------------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV DEV_NAME=rosdev
ENV ROS_DISTRO=galactic
ENV GROUP_NAME=ros
ENV WS_NAME=colcon_ws

# --------------------------------------------------------------------
# Base system setup (Linux utilities, Python, etc.)
# --------------------------------------------------------------------
RUN echo "Set disable_coredump false" >> /etc/sudo.conf && \
    # 🔧 FIX: remove stale Google Chrome repo that breaks apt-get update
    rm -f /etc/apt/sources.list.d/google-chrome.list && \
    # Update and install essential packages
    apt-get update -q && \
    apt-get upgrade -yq && \
    apt-get install -yq \
        wget \
        curl \
        git \
        build-essential \
        vim \
        sudo \
        gnupg2 \
        lsb-release \
        locales \
        bash-completion \
        tzdata \
        gosu \
        python3-argcomplete \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------
# Install ROS 2 Galactic (base packages, build tools)
# --------------------------------------------------------------------
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
        /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update -q && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-vcstool \
    && rm -rf /var/lib/apt/lists/* && rm /etc/apt/sources.list.d/ros2-latest.list

# --------------------------------------------------------------------
# Create a non-root ROS user and copy the workspace
# --------------------------------------------------------------------
RUN useradd --create-home --home-dir /home/${DEV_NAME} --shell /bin/bash --user-group --groups adm,sudo ${DEV_NAME} && \
    echo "$DEV_NAME:$DEV_NAME" | chpasswd && \
    echo "$DEV_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

COPY --chown=${DEV_NAME} colcon_ws /home/${DEV_NAME}/colcon_ws
COPY ros2-setup.bash /bin/ros2-setup.bash

# --------------------------------------------------------------------
# Initialize rosdep, build workspace dependencies
# --------------------------------------------------------------------
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
        /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update -q && \
    rosdep init && \
    chmod +x /bin/ros2-setup.bash && \
    gosu ${DEV_NAME} /bin/ros2-setup.bash && \
    runuser -u ${DEV_NAME} ros2-setup.bash && \
    rm /bin/ros2-setup.bash && \
    rm -rf /var/lib/apt/lists/* && rm /etc/apt/sources.list.d/ros2-latest.list

# --------------------------------------------------------------------
# Auto-source ROS setup on shell startup
# --------------------------------------------------------------------
RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${DEV_NAME}/.bashrc && \
    echo ". /home/${DEV_NAME}/${WS_NAME}/install/local_setup.bash" >> /home/${DEV_NAME}/.bashrc

# --------------------------------------------------------------------
# Set TurtleBot3 model environment variable for SLAM demo
# --------------------------------------------------------------------
ENV TURTLEBOT3_MODEL=waffle_pi

# --------------------------------------------------------------------
# Final environment setup
# --------------------------------------------------------------------
# This ensures Docker knows the default user inside the container.
ENV USER=${DEV_NAME}

# You can now build this with:
# docker build --no-cache -t unity-robotics:nav2-slam-example .
#
# And run it using:
# docker run --name nav2slam --rm -it ^
#  -p 10000:10000 ^
#  -p 6080:80 ^
#  -v "C:/UnityRoboticsProjects/Robotics-Nav2-SLAM-Example/ros2_docker/colcon_ws:/root/colcon_ws" ^
#  unity-robotics:nav2-slam-example
