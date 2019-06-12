# robowflex 💪🤖
The Better Robot Benching Framework™.
Documentation for this project can be read online at [GitHub pages](https://kavrakilab.github.io/robowflex/).

## robowflex_library
A library that simplifies using [_MoveIt!_](https://github.com/ros-planning/moveit) in applications.
Examples of basic motion planning, benchmarking of planning requests, and input/output can be found in the `robowflex_library/scripts` directory.

## robowflex_visualization
Python scripts for visualizing robots and motion plans in [Blender](https://www.blender.org/).
See [`robowflex_visualization/README.md`](robowflex_visualization/README.md) for more information on how to use.

## robowflex_ompl
An optionally compiled library component that adds more direct access to [OMPL](http://ompl.kavrakilab.org/) through a new `robowflex::Planner`.
Requires [`moveit_planners_ompl`](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_planners/ompl), from [_MoveIt!_](https://github.com/ros-planning/moveit).
Some basic examples of how to use the new planner are in `robowflex_ompl/scripts`.

## robowflex_tesseract
An optionally compiled library component that adds support for [tesseract](https://github.com/ros-industrial-consortium/tesseract)-based planners.
Currently, only [OMPL](http://ompl.kavrakilab.org/)-based planning is available.
Requires both [tesseract](https://github.com/ros-industrial-consortium/tesseract) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros).
Some basic examples of how to use the new planner(s) are in `robowflex_tesseract/scripts`.

## robowflex_movegroup
A library component with helper classes and functions to interact with a `move_group` process being used for motion planning.
Scenes can be pushed and pulled and trajectories can be executed with this component through `move_group`.

## tmpack
Task and motion planning using `robowflex` as the motion planning layer.

## robowflex_doc
Documentation for all library and module components in robowflex (sans `tmpack`).
Documentation is automatically generated using [Doxygen](http://www.stack.nl/~dimitri/doxygen/), and is placed in `${CATKIN_DEVEL_PREFIX}/share/robowflex_doc/doc/index.html`. The online version exists [here](https://kavrakilab.github.io/robowflex/).


# Installation Instructions

For a new installation on an Ubuntu 18.04 machine without ROS already installed.

First, install ROS following the directions here (example copied below):
http://wiki.ros.org/melodic/Installation/Ubuntu

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
rosdep update
source /opt/ros/melodic/setup.bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Then install OMPL from source or binary.

```
sudo apt install libompl-dev
```

Then:
```
sudo apt install python-catkin-tools
sudo apt install ros-melodic-moveit-ros-planning
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/melodic/setup.bash (if you haven't already)
catkin config --init
cd src
git clone http://wiki.ros.org/melodic/Installation/Ubuntu
catkin build
```


