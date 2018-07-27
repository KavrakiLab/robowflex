# robowflex 💪🤖
The Better Robot Benching Framework™.

## robowflex_doc
Documentation for all library and module components in robowflex.
Documentation is automatically generated using [Doxygen](http://www.stack.nl/~dimitri/doxygen/), and is placed in `${CATKIN_DEVEL_PREFIX}/share/robowflex_doc/doc/index.html`.

## robowflex_library
A library that simplifies using [_MoveIt!_](https://github.com/ros-planning/moveit) in applications.
Examples of basic motion planning, benchmarking of planning requests, and input/output can be found in the `robowflex_library/scripts` directory.

## robowflex_visualization
Python scripts for visualizing robots and motion plans in [Blender](https://www.blender.org/).
See `robowflex_visualization/README.md` for more information on how to use.

## robowflex_ompl
An optionally compiled library component that adds more direct access to [OMPL](http://ompl.kavrakilab.org/) through a new `robowflex::Planner`.
Requires [`moveit_planners_ompl`](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_planners/ompl), from [_MoveIt!_](https://github.com/ros-planning/moveit).
Some basic examples of how to use the new planner are in `robowflex_ompl/scripts`.

## robowflex_tesseract
An optionally compiled library component that adds support for [tesseract](https://github.com/ros-industrial-consortium/tesseract)-based planners.
Currently, only [OMPL](http://ompl.kavrakilab.org/)-based planning is available.
Requires both [tesseract](https://github.com/ros-industrial-consortium/tesseract) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros).
Some basic examples of how to use the new planner(s) are in `robowflex_tesseract/scripts`.

## tmpack
Task and motion planning using `robowflex` as the motion planning layer.
