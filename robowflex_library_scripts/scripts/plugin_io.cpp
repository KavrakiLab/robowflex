/* Author: Zachary Kingston */

#include <moveit/planning_request_adapter/planning_request_adapter.h>

#include <robowflex_library/io/plugin.h>
#include <robowflex_library/util.h>

using namespace robowflex;

/* \file plugin_io.cpp
 * Demonstrates how to use the plugin loader helper class to load some MoveIt
 * plugins.
 */

int main(int argc, char **argv)
{
    ROS ros(argc, argv);

    auto plugin1 = IO::PluginManager::load<planning_request_adapter::PlanningRequestAdapter>(  //
        "moveit_core", "default_planner_request_adapters/AddTimeParameterization");

    auto plugin2 = IO::PluginManager::load<planning_request_adapter::PlanningRequestAdapter>(  //
        "moveit_core", "default_planner_request_adapters/FixStartStateBounds");

    auto plugin3 = IO::PluginManager::load<planning_request_adapter::PlanningRequestAdapter>(  //
        "moveit_core", "default_planner_request_adapters/FixStartStateCollision");

    return 0;
}
