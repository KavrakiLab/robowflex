FROM ros:melodic-ros-base
MAINTAINER Zachary Kingston zak@rice.edu

# Download Dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    apt-add-repository ppa:dartsim/ppa && \
    apt-get update && \
    apt-get install -y \
      build-essential  \
      wget \
      pkg-config \
      python-catkin-tools \ 
      castxml \
      cmake \
      libboost-all-dev \
      libeigen3-dev \
      libexpat1 \
      libflann-dev \
      libode-dev \
      libtinfo5 \
      libtriangle-dev \
      python3-dev \
      python3-numpy \
      python3-pip \
      libyaml-cpp-dev \
      libhdf5-dev \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-fetch-ros \
      libdart6-all-dev

# Setup Catkin Workspace
RUN mkdir -p ws/src
WORKDIR /ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --limit-status-rate 0.001 --no-notify
WORKDIR /

# Get & Install OMPL
WORKDIR /ws/src
RUN wget -O - https://github.com/ompl/ompl/archive/1.5.0.tar.gz | tar zxf -
WORKDIR /ws
RUN catkin build ompl --start-with ompl
WORKDIR /

# Build Robowflex
WORKDIR /ws/src
ADD ./CMakeModules CMakeModules

# Main Library
ADD ./robowflex_library robowflex_library
RUN catkin build robowflex_library --start-with robowflex_library

# OMPL Module
ADD ./robowflex_ompl robowflex_ompl
RUN catkin build robowflex_ompl --start-with robowflex_ompl

# Movegroup Module
ADD ./robowflex_movegroup robowflex_movegroup
RUN catkin build robowflex_movegroup --start-with robowflex_movegroup

# Tesseract Module
ADD ./robowflex_tesseract robowflex_tesseract
RUN catkin build robowflex_tesseract --start-with robowflex_tesseract

# DART Module
ADD ./robowflex_dart robowflex_dart
RUN catkin build robowflex_dart --start-with robowflex_dart
WORKDIR /