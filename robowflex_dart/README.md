# Robowflex Dart

Modeling and planning through [DART (Dynamic Animation and Robotics Toolkit)](https://dartsim.github.io/).
Robots (`robowflex::darts::Robot`) and scene geometry (`robowflex::darts::Structure`) are added to a world (`robowflex::darts::World`), which is then used as the base for planning.
An [OMPL](http://ompl.kavrakilab.org/) state space is provided for worlds (`robowflex::darts::StateSpace`) which can control any joint in the world, for multiple robots.
Constraints and inverse kinematics are given through a Task Space Region specification (`robowflex::darts::TSR`).
For convenience, a helper class (`robowflex::darts::PlanBuilder`) makes it easy to specify motion planning queries.

# Scripts

There are a few example scripts to demonstrate the module, in the `scripts` directory.
- `fetch_plan.cpp`: Plan for 1 to 4 Fetch robots. Demonstrates how to do multi-robot planning by cloning robots (`robowflex::darts::Robot::clone()`) and adding more groups to the planning group.
- `fetch_robowflex_plan.cpp`: Constrained planning for a Fetch robot, where that Fetch robot is loaded via `robowflex_library` as a _MoveIt_ robot.

# Installation Instructions

DART can either be installed via a PPA or be added as a catkin package in your local workspace.
See [DART's own installation instructions for more details](https://dartsim.github.io/install_dart_on_ubuntu.html#install-dart).

## PPA Installation

To install DART via PPA, simply do the following:

```sh
sudo apt-add-repository ppa:dartsim/ppa
sudo apt-get update  # not necessary since Bionic
```

After the PPA is installed, then get DART:
```sh
sudo apt-get install libdart6-all-dev
```

## Source Installation

Install DART's dependencies:
```sh
sudo apt-get install \
  libeigen3-dev \
  libassimp-dev \
  libccd-dev \
  libfcl-dev \
  libboost-regex-dev \
  libboost-system-dev \
  libopenscenegraph-dev \
  libnlopt-dev \
  coinor-libipopt-dev \
  libbullet-dev \
  libode-dev \
  liboctomap-dev \
  libflann-dev \
  libtinyxml2-dev \
  liburdfdom-dev \
  libxi-dev \
  libxmu-dev \
  freeglut3-dev \
  libopenscenegraph-dev
```

Then, add DART to your local catkin workspace:
```sh
git clone -b v6.10.0 git://github.com/dartsim/dart.git
```

## OMPL Installation

The DART module relies on recent versions of OMPL (> 1.4.0).
If you are using an old OMPL (e.g., on Kinetic), either download the source into your local catkin workspace:
```sh
wget -O - https://github.com/ompl/ompl/archive/1.5.0.tar.gz | tar zxf -
```
Or clone from the repository:
```sh
git clone git:://github.com/ompl/ompl.git
```
