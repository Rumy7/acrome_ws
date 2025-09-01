FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Temel bağımlılıklar
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release

# Çakışan ROS kaynaklarını sil
RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.sources

# ROS 2 repository ekle
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update

# Çalışma dizini
WORKDIR /root/acrome_ws

# ROS Jazzy paketlerini kur
RUN apt-get install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-velocity-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Kaynak dosyaları
COPY ./src ./src

# rosdep opsiyonel
RUN rosdep update || true
