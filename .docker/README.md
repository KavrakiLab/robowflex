# Robowflex Docker Containers

Here are docker containers for compiling Robowflex, for use locally or within CI.
Each container comes with a `build-*.sh` script that runs the `docker build` command.
The current list of containers are:

## robowflex/kinetic
Builds the Robowflex code on ROS Kinetic, Ubuntu 16.04.
Builds:
- `robowflex_library`
- `robowflex_ompl`
- `robowflex_movegroup`

## robowflex/melodic
Builds the Robowflex code on ROS Melodic, Ubuntu 18.04.
Also builds the DART module, which is built with OMPL from source (1.5.0).
Builds:
- `robowflex_library`
- `robowflex_ompl`
- `robowflex_movegroup`
- `robowflex_dart`

## robowflex/noetic
Builds the Robowflex code on ROS Noetic, Ubuntu 20.04.
Also builds the DART module, which is built with OMPL from source (1.5.0).
Builds:
- `robowflex_library`
- `robowflex_ompl`
- `robowflex_movegroup`
- `robowflex_dart`

## robowflex/melodic-source
Builds the Robowflex code on ROS Melodic, Ubuntu 18.04.
Also builds OMPL (1.5.0) and MoveIt (`master` branch) from source.
Builds:
- `robowflex_library`
- `robowflex_ompl`
- `robowflex_movegroup`

## robowflex/tesseract
Builds the Robowflex code on ROS Melodic, Ubuntu 18.04.
In addition, builds the Tesseract module.
Builds:
- `robowflex_library`
- `robowflex_tesseract`
