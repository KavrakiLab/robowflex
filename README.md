# Robowflex 💪🤖
Making MoveIt Easy.
Documentation for this project can be read online at [GitHub pages](https://kavrakilab.github.io/robowflex/).
If you use Robowflex, please use the following citation for the [associated paper](https://arxiv.org/pdf/2103.12826.pdf):
```bibtex
@article{Kingston2021,
  author = {Kingston, Zachary and Kavraki, Lydia E.},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems},
  title = {{Robowflex}: Robot Motion Planning with {MoveIt} Made Easy},
  year = {2022},
  month = {10}
}
```

## What is Robowflex?

Robowflex is primarily a wrapper around components from other robotics libraries (in particular, components from _MoveIt_).
The name "Robowflex" is a portmanteau of "Robot" and "Bowflex", as Robowflex was intended just to be better "bench"-marking software (as in "bench press") than the rest.
However, the software needed for doing benchmarking was the same as the software needed for running motion planning in "isolation" from a ROS system while still using ROS components, useful for many other things other than benchmarking.
Thus, the project has spun out and become a toolbox for motion planning using real robots that are _MoveIt_ enabled.

The high-level goals of the project are as follows:
- Provide an easy to use interface to complex robotics libraries by brushing cruft and idiosyncrasies under the rug.
- Provide access to all underlying capabilities of the libraries so users are not hampered by Robowflex in any way.
- Remain untied as possible to the greater ROS system, and support all distros from Kinetic onward.

## Build Status
| Workflow      | Status                                                                                                                                                                              |
|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Documentation | ![Documentation](https://github.com/KavrakiLab/robowflex/workflows/Documentation/badge.svg)                                                                                         |
| Linting       | ![Linting](https://github.com/KavrakiLab/robowflex/workflows/Linting/badge.svg)                                                                                                     |
| Kinetic       | ![Build Kinetic](https://github.com/KavrakiLab/robowflex/workflows/Build%20Kinetic/badge.svg)                                                                                       |
| Melodic       | ![Build Melodic](https://github.com/KavrakiLab/robowflex/workflows/Build%20Melodic/badge.svg)                                                                                       |
| Noetic        | [![Build Noetic](https://github.com/KavrakiLab/robowflex/actions/workflows/build-noetic.yml/badge.svg)](https://github.com/KavrakiLab/robowflex/actions/workflows/build-noetic.yml) |

## robowflex_library
A library that simplifies using [_MoveIt_](https://github.com/ros-planning/moveit) in applications.
Examples of basic motion planning, benchmarking of planning requests, and input/output can be found in the `robowflex_library/scripts` directory.

# Optional Robowflex Modules
Robowflex also comes with many optional modules that provide specific auxiliary library support through the standard Robowflex interface.
The modules are only compiled if the required dependencies are installed.

## robowflex_visualization
Python scripts for visualizing robots and motion plans in [Blender](https://www.blender.org/).
See [`robowflex_visualization/README.md`](robowflex_visualization/README.md) for more information on how to use.

## robowflex_ompl
An optionally compiled library component that adds more direct access to [OMPL](http://ompl.kavrakilab.org/) through a new `robowflex::Planner`.
Requires [`moveit_planners_ompl`](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_planners/ompl), from [_MoveIt_](https://github.com/ros-planning/moveit).
Some basic examples of how to use the new planner are in `robowflex_ompl/scripts`.

## robowflex_tesseract
An optionally compiled library component that adds support for [tesseract](https://github.com/ros-industrial-consortium/tesseract)-based planners.
Requires both [tesseract](https://github.com/ros-industrial-consortium/tesseract) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros).
Some basic examples of how to use the new planner(s) are in `robowflex_tesseract/scripts`.

## robowflex_movegroup
A library component with helper classes and functions to interact with a `move_group` process being used for motion planning.
Scenes can be pushed and pulled and trajectories can be executed with this component through `move_group`.

## robowflex_dart
A optionally compiled library that adds support for modeling and planning through [DART (Dynamic Animation and Robotics Toolkit)](https://dartsim.github.io/).
There are features for loading robots just through DART or by converting __MoveIt_ robots into the DART representation.
Motion planning is supported through [OMPL](http://ompl.kavrakilab.org/).
This module offers easy multi-robot motion planning through composing complex worlds with multiple robots.
Additionally, this module has [manifold-constrained motion planning](http://ompl.kavrakilab.org/constrainedPlanning.html) with a Task Space Region constraint specification.

## robowflex_resources
[robowflex_resources](https://github.com/KavrakiLab/robowflex_resources) is a separate repository that contains the MoveIt configuration and kinematic description for robots used in example scripts.

# Installation Instructions

Robowflex is supported on many ROS platforms, from Kinetic on Ubuntu 14.04 to Melodic on 18.04.
Here are some bare-bones installation instructions to get up and running on a new Ubuntu 18.04 machine without ROS already installed.
This will only enable the core modules to be built, as some modules require special packages to be installed.

First, [install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) following the directions copied below:

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
rosdep update
source /opt/ros/melodic/setup.bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

We recommend `catkin-tools` to build your ROS workspace:
```sh
sudo apt install python-catkin-tools
```

Install _MoveIt_ and other required ROS packages:
```sh
sudo apt install ros-melodic-moveit ros-melodic-geometry
```

Finally, create a workspace with Robowflex inside:
```sh
cd ~
mkdir -p rb_ws/src
cd rb_ws
source /opt/ros/melodic/setup.bash # if you haven't already
catkin config --init
cd src
git clone https://github.com/KavrakiLab/robowflex.git
catkin build
```

To try out a demo script, you first need a robot description.
The easiest to try is the _Fetch_ robot, either by debian or source:
```sh
# Debian
sudo apt install ros-melodic-fetch-ros

# Or, Source
cd ~/rb_ws/src
git clone https://github.com/fetchrobotics/fetch_ros
catkin build
```

After the workspace is built, source and run a demo:
```sh
cd ~/rb_ws
source ./devel/setup.bash
rosrun robowflex_library fetch_test
```

## Code Formatting
All C++ code should be formatted with `clang-format`.
Use the `format.sh` script to automatically format the code base.

## Applications 
External projects that make use of Robowflex:
- Learning and Motion Planning Algorithms: https://github.com/KavrakiLab/pyre
