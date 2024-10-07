FROM ros:noetic

RUN apt-get update && apt-get install -y \ 
    locales \
    curl 

ARG DEBIAN_FRONTEND=noninteractive

ENV LANG=en_US.UTF-8 
ENV LANGUAGE=en_US:en 
ENV LC_ALL=en_US.UTF-8

RUN locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

RUN apt install -y software-properties-common 

RUN add-apt-repository universe 

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt upgrade -y\
    && apt install -y ros-galactic-desktop 

ENV ROS1_INSTALL_PATH=/opt/ros/noetic 
ENV ROS2_INSTALL_PATH=/opt/ros/galactic 

RUN apt install -y \
    ros-galactic-ros1-bridge \
    ros-galactic-rosbag2-storage-mcap \
    ros-noetic-velodyne*
