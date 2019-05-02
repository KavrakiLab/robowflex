/* Author: Zachary Kingston */

#include <moveit/planning_request_adapter/planning_request_adapter.h>

#include <robowflex_library/util.h>
#include <robowflex_library/io/plugin.h>

using namespace robowflex;

int main(int argc, char **argv)
{
#if ROBOWFLEX_AT_LEAST_MELODIC
    ROS ros(argc, argv);

    auto plugin1 = IO::PluginManager::load<planning_request_adapter::PlanningRequestAdapter>(  //
        "moveit_core", "default_planner_request_adapters/AddTimeParameterization");

    auto plugin2 = IO::PluginManager::load<planning_request_adapter::PlanningRequestAdapter>(  //
        "moveit_core", "default_planner_request_adapters/FixStartStateBounds");

    auto plugin3 = IO::PluginManager::load<planning_request_adapter::PlanningRequestAdapter>(  //
        "moveit_core", "default_planner_request_adapters/FixStartStateCollision");

    return 0;
#else
    return -1;
#endif
}
