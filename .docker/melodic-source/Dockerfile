FROM ros:melodic-ros-base
MAINTAINER Zachary Kingston zak@rice.edu

# Set Arguments
ENV CAKTIN_BUILD_ARGS="--limit-status-rate 0.001 --no-notify"
ENV MOVEIT_BRANCH="master"

# Download Dependencies
RUN apt-get update && \
    apt-get install -y \
      build-essential wget pkg-config python-catkin-tools python-wstool cmake git \
      libboost-all-dev libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libhdf5-dev

# Setup Catkin Workspace
RUN mkdir -p ws/src
WORKDIR /ws/src
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex_resources
WORKDIR /ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build ${CATKIN_BUILD_ARGS}

# Get & Install OMPL
WORKDIR /ws/src
RUN wget -O - https://github.com/ompl/ompl/archive/1.5.0.tar.gz | tar zxf -
RUN catkin build ompl --start-with ompl $CAKTIN_BUILD_ARGS

# Get & Install MoveIt from Source
WORKDIR /ws
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/${MOVEIT_BRANCH}/moveit.rosinstall && \
    wstool update -t src && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
RUN catkin build $CAKTIN_BUILD_ARGS

# Build Robowflex
WORKDIR /ws/src
ADD ./CMakeModules CMakeModules

# Main Library
ADD ./robowflex_library robowflex_library
RUN catkin build robowflex_library --start-with robowflex_library $CAKTIN_BUILD_ARGS

# OMPL Module
ADD ./robowflex_ompl robowflex_ompl
RUN catkin build robowflex_ompl --start-with robowflex_ompl $CAKTIN_BUILD_ARGS

# Movegroup Module
ADD ./robowflex_movegroup robowflex_movegroup
RUN catkin build robowflex_movegroup --start-with robowflex_movegroup $CAKTIN_BUILD_ARGS
WORKDIR /