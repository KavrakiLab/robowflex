# Robowflex {#home}

Making MoveIt Easy!
An overview of what this project is can be found in the [design description](\ref design).
Examples of basic motion planning, benchmarking of planning requests, input/output, and more can be found in [scripts](\ref scripts).

## robowflex_library
A library that simplifies using [_MoveIt!_](https://github.com/ros-planning/moveit) in applications.
This is the core library used by all other components in Robowflex.
Take a look at the [design description](\ref design) for more information on the core library.

## Optional Components

There are also a few optional modules that are compiled if the requisite libraries are available. 
These will only compile if the necessary components are found.

### robowflex_ompl
An optionally compiled library component that adds more direct access to [OMPL](http://ompl.kavrakilab.org/) through a new `robowflex::Planner`.
Requires [`moveit_planners_ompl`](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_planners/ompl), from [_MoveIt!_](https://github.com/ros-planning/moveit).

### robowflex_tesseract
An optionally compiled library component that adds support for [tesseract](https://github.com/ros-industrial-consortium/tesseract)-based planners.
Currently, only [OMPL](http://ompl.kavrakilab.org/)-based planning is available.
Requires both [tesseract](https://github.com/ros-industrial-consortium/tesseract) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros).

### robowflex_movegroup
A library component with helper classes and functions to interact with a `move_group` process being used for motion planning.
Scenes can be pushed and pulled and trajectories can be executed with this component through `move_group`.

### robowflex_dart
A optionally compiled library that adds support for modeling and planning through [DART (Dynamic Animation and Robotics Toolkit)](https://dartsim.github.io/).
There are features for loading robots just through DART or by converting __MoveIt__ robots into the DART representation.
Motion planning is supported through [OMPL](http://ompl.kavrakilab.org/).
This module offers easy multi-robot motion planning through composing complex worlds with multiple robots.
Additionally, this module has [manifold-constrained motion planning](http://ompl.kavrakilab.org/constrainedPlanning.html) with a Task Space Region constraint specification.

### robowflex_visualization
Python scripts for visualizing robots and motion plans in [Blender](https://www.blender.org/).
See [the readme](md__home_runner_work_robowflex_robowflex_robowflex_visualization_README.html) for more information on how to use.
