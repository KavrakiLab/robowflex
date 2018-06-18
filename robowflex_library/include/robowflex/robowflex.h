#ifndef ROBOWFLEX_
#define ROBOWFLEX_

#include <fstream>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>

// #include <moveit/ompl_interface/ompl_interface.h>
// #include <moveit/ompl_interface/model_based_planning_context.h>

#define ROBOWFLEX_AT_LEAST_INDIGO ROS_VERSION_MINIMUM(1, 11, 0)
#define ROBOWFLEX_AT_LEAST_LUNAR ROS_VERSION_MINIMUM(1, 12, 0)
#define ROBOWFLEX_AT_LEAST_KINETIC ROS_VERSION_MINIMUM(1, 13, 0)
#define ROBOWFLEX_AT_LEAST_MELODIC ROS_VERSION_MINIMUM(1, 14, 0)

#include "robowflex/util.h"
#include "robowflex/geometry.h"
#include "robowflex/scene.h"
#include "robowflex/planning.h"
#include "robowflex/benchmarking.h"

#include "robowflex/yaml.h"

#endif
