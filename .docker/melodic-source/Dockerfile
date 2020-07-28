FROM ros:melodic-ros-base
MAINTAINER Zachary Kingston zak@rice.edu

# Download Dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-add-repository ppa:dartsim/ppa && \
    apt-get update && \
    apt-get install -y \
      build-essential wget pkg-config python-catkin-tools python-wstool cmake \
      libboost-all-dev libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libhdf5-dev \
      ros-${ROS_DISTRO}-geometry \
      libdart6-all-dev

# Setup Catkin Workspace
RUN mkdir -p ws/src
WORKDIR /ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --limit-status-rate 0.001 --no-notify
WORKDIR /

# Get & Install MoveIt! from Source
WORKDIR /ws
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall && \
    wstool update -t src && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
RUN catkin build --limit-status-rate 0.001 --no-notify
WORKDIR /

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