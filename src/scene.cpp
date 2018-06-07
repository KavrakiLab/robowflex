#include "robowflex.h"

using namespace robowflex;

Scene::Scene(const robot_model::RobotModelConstPtr model) : scene_(new planning_scene::PlanningScene(model))
{
}
