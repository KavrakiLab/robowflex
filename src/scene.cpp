#include "robowflex.h"

using namespace robowflex;

Scene::Scene(Robot &robot) : scene_(new planning_scene::PlanningScene(robot.getModel()))
{
}

robot_state::RobotState &Scene::getCurrentState()
{
    return scene_->getCurrentStateNonConst();
}
