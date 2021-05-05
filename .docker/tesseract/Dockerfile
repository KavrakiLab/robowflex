FROM ros:melodic-ros-base
MAINTAINER Zachary Kingston zak@rice.edu

ENV TESSERACT_BRANCH="kinetic-devel"

# Download Dependencies
RUN apt-get update && \
    apt-get install -y \
      build-essential git wget pkg-config python-catkin-tools python-rosdep cmake \
      libboost-all-dev libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libhdf5-dev \
      ros-${ROS_DISTRO}-moveit

# Setup rosdep
RUN rosdep update

# Setup Catkin Workspace
RUN mkdir -p ws/src
WORKDIR /ws/src
RUN git clone --depth 1 https://github.com/KavrakiLab/robowflex_resources
WORKDIR /ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --limit-status-rate 0.001 --no-notify

# Setup Tesseract
WORKDIR /ws/src/
RUN git clone --recursive --single-branch --branch ${TESSERACT_BRANCH} --depth 1 https://github.com/ros-industrial-consortium/tesseract.git
RUN git clone --recursive --single-branch --branch ${TESSERACT_BRANCH} --depth 1 https://github.com/ros-industrial-consortium/trajopt_ros.git
WORKDIR /ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin build --limit-status-rate 0.001 --no-notify

# Build Robowflex
WORKDIR /ws/src
ADD ./CMakeModules CMakeModules

# Main Library
ADD ./robowflex_library robowflex_library
RUN catkin build robowflex_library --start-with robowflex_library --limit-status-rate 0.001 --no-notify

# Tesseract Module
ADD ./robowflex_tesseract robowflex_tesseract
RUN catkin build robowflex_tesseract --start-with robowflex_tesseract --limit-status-rate 0.001 --no-notify
WORKDIR /