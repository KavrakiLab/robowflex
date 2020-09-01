FROM ros:indigo-ros-base
MAINTAINER Zachary Kingston zak@rice.edu

# Download Dependencies
# Need to install more recent g++ for C++11 compatibility
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-add-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y \
      build-essential g++-4.9 pkg-config python-catkin-tools cmake \
      libboost-all-dev libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libhdf5-dev \
      ros-${ROS_DISTRO}-moveit && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 50

# Setup Catkin Workspace
RUN mkdir -p ws/src
WORKDIR /ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --limit-status-rate 0.001 --no-notify

# Install TF2 - not included in Indigo repositories
WORKDIR /
RUN git clone --single-branch --branch indigo-devel --depth 1 https://github.com/ros/geometry2.git
RUN ln -s /geometry2/tf2 /ws/src/tf2 && ln -s /geometry2/tf2_eigen /ws/src/tf2_eigen
WORKDIR /ws
RUN catkin build --limit-status-rate 0.001 --no-notify

# Build Robowflex
WORKDIR /ws/src
ADD ./CMakeModules CMakeModules

# Main Library
ADD ./robowflex_library robowflex_library
RUN catkin build robowflex_library --start-with robowflex_library --limit-status-rate 0.001 --no-notify

# OMPL Module
ADD ./robowflex_ompl robowflex_ompl
RUN catkin build robowflex_ompl --start-with robowflex_ompl --limit-status-rate 0.001 --no-notify

# Movegroup Module
ADD ./robowflex_movegroup robowflex_movegroup
RUN catkin build robowflex_movegroup --start-with robowflex_movegroup --limit-status-rate 0.001 --no-notify
WORKDIR /