# Robowflex Design Notes {#design}

## Introduction (What is Robowflex?)

Robowflex is primarily a wrapper around components from other robotics libraries (in particular, components from _MoveIt!_).
The name "Robowflex" is a portmanteau of "Robot" and ["Bowflex"](https://en.wikipedia.org/wiki/Bowflex), as Robowflex was intended just to be better "bench"-marking software (as in ["bench press"](https://en.wikipedia.org/wiki/Bench_press)) than the rest.
However, the software needed for doing benchmarking was the same as the software needed for running motion planning in "isolation" from a ROS system while still using ROS components, useful for many other things other than benchmarking.
Thus, the project has spun out and become a toolbox for motion planning using real robots that are _MoveIt!_ enabled.

The high-level goals of the project are as follows:
1. Provide an easy to use interface to complex robotics libraries by brushing the cruft and idiosyncrasies under the rug.
2. Provide access to all underlying capabilities of the libraries so users are not hampered by Robowflex in any way.
3. Remain untied as possible to the greater ROS system, and support all distros from Kinetic onward.

This document provides a high-level overview of the design and API of the `robowflex_library`, as well as some of the auxiliary components.
Unless otherwise noted, the class or method will be in `robowflex_library`.

## Components

There are only a few core components in Robowflex that are necessary for planning.
A brief overview of them is provided below.
Take a look at the [scripts](scripts.html) to see the components in action.

- robowflex::Robot: A representation of a robot.
This requires a URDF and SRDF, as well as _MoveIt!_'s joint limits and kinematics plugin information (at least, for _MoveIt!_-based planning).
Along with loading information about the robot's geometry, this class provides some helper methods for setting a robot's state.
Each robot has a scratch state that can be modified.
Under the hood, the robot is represented with _MoveIt!_'s `robot_model::RobotModel`, and the scratch state is a `robot_state::RobotState`.
See the detailed description in Robot for more information.

- robowflex::Scene: A representation of a planning scene. 
Requires a Robot to initialize with. 
Collision objects can be added and moved (as Geometry, see below) and objects can be attached to the robot used for planning (after they have been added to the scene).
Under the hood, the scene is represented with _MoveIt!_'s `planning_scene::PlanningScene`.

- robowflex::Geometry: Container of both solid primitives and mesh geometry used in planning requests and collision objects in a scene.
A few static methods `Geometry::make*` are provided to easily create instances of geometry.
The class wraps a `shapes::Shape` and `bodies::Body`, which are used many places in _MoveIt!_.

- robowflex::Planner: A motion planner that can compute a plan for a Robot in a Scene.
A few default implementations are provided, such as the default OMPL planning pipeline plugin (OMPL::OMPLPipelinePlanner).
However, there are many other Planners available, such as the PoolPlanner which enables threaded planning requests, `robowflex_ompl` which gives direct access to the OMPL state space used in _MoveIt!_, and `robowflex_tesseract` provides a planning interface through [tesseract](https://github.com/ros-industrial-consortium/tesseract) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros).

- robowflex::MotionRequestBuilder: A helper class to help build a motion planning request for a planner.
Simplifies the design of complex goal and path constraints, as well as setting start and goal states.

- robowflex::Benchmarker: A utility class that simplifies running a set benchmarks.
Different requests (A scene, planner, and request) can be added to a list of experiments to run, along with a set of metrics to compute about the results.
Then, the benchmark can be run with multiple BenchmarkOutputter classes, which dump results somehow.

- robowflex::BenchmarkOutputter: A class that writes benchmarking results to something.
A few useful outputters are provided by default, such as the OMPLBenchmarkOutputter, which dumps results into an OMPL benchmarking log format that can be read (after being made into a database) on [Planner Arena](planner_arena.org).

## Connective Tissue

One of the core challenges that Robowflex tackles is the issues of input / output in a ROS system (see IO).
Robowflex provides a few useful things on this front:
- Automatic resolution of file paths and ROS package paths (IO::resolvePath() and IO::resolvePackage()).
- Open XML and unprocessed xacro files to strings (IO::loadXMLToString()).
- Broad YAML conversion for ROS messages (see [`yaml.h`](yaml_8h_source.html) and `io/yaml.h`) compatible with the output dumped by ROS Python and `rostopic echo`.
Many of the components provide methods that serialize / deserialize YAML files.
- There are also many useful conversions and transformation-related methods in `tf.h`.
- ROS bag file reading / writing (see robowflex::IO::Bag).
- HDF5 file reading (see robowflex::IO::HDF5File).
- Helpful live visualization in RViz through robowflex::IO::RVIZHelper.
Offline visualization can be done with Blender through `robowflex_visualization`.
See the [readme](robowflex_visualization/README.html) for more details.

Additionally, there are a few implementations of robowflex::Robot for some commonly used robots, such as robowflex::UR5Robot, robowflex::FetchRobot, and robowflex::R2Robot.

## Compatibility

Robowflex strives to maintain compatibility with all commonly used ROS distributions, from Kinetic to Noetic.
Indigo support was dropped due to lack of ROS Docker support.
To this end, there are many adapters and internal constructs so that users can run Robowflex in any of these environments without modification.
However, there are some things to note about how various internal APIs have changed and how this affects behavior:
- robowflex::ROS will attempt to spin up an instance of `rosmaster` if one is not already running only on Melodic onward, due to a dependency of how this is implemented on Boost 1.64.
- _MoveIt!_ changed from using `Eigen::Affine3d` as the representation of transformation matrices to `Eigen::Isometry3d` in Melodic in version 0.10.6. To account for this, we provide `robowflex::RobotPose` which is a type alias for the correct matrix representation.
- YAML output does not "flow" output on Indigo for more concise files. 

Note that there are other internal functions that account for API differences between ROS versions, but they are not relevant to user code.
There are macros in [`macros.h`](macros_8h_source.html) that allow for conditional compilation on versions.

## Documentation

See [here](@ref doc) and [here](@ref doc) for how documentation is generated.
This documents all code in the other Robowflex projects, as well as their `README.md` files if they exist (see the `add_doc` macro in `.docs/CMakeLists.txt`).
Additional written documentation is in the `.docs/markdown` folder.

