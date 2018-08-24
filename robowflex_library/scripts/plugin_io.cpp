/* Author: Zachary Kingston */

#include <moveit/planning_request_adapter/planning_request_adapter.h>

#include <robowflex_library/io/plugin.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    auto &manager = IO::PluginManager::getInstance();
    auto plugin = manager.load<planning_request_adapter::PlanningRequestAdapter>(  //
        "default_planner_request_adapters/AddTimeParameterization");

    return 0;
}
