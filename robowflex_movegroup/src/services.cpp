/* Author: Zachary Kingston */

// #include <sensor_msgs/JointState.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <moveit_msgs/ExecuteTrajectoryResult.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <robowflex_movegroup/services.h>

using namespace robowflex;
using namespace robowflex::movegroup;

const std::string MoveGroupHelper::MOVE_GROUP{"/move_group"};
const std::string MoveGroupHelper::GET_SCENE{"get_planning_scene"};
const std::string MoveGroupHelper::APPLY_SCENE{"apply_planning_scene"};
const std::string MoveGroupHelper::EXECUTE{"execute_trajectory"};

MoveGroupHelper::MoveGroupHelper(const std::string &move_group)
  : nh_("~")
  , goal_sub_(nh_.subscribe(move_group + "/goal", 10, &MoveGroupHelper::moveGroupGoalCallback, this))
  , result_sub_(nh_.subscribe(move_group + "/result", 10, &MoveGroupHelper::moveGroupResultCallback, this))
  , gpsc_(nh_.serviceClient<moveit_msgs::GetPlanningScene>(GET_SCENE, true))
  , apsc_(nh_.serviceClient<moveit_msgs::ApplyPlanningScene>(APPLY_SCENE, true))
  , eac_(nh_, EXECUTE, false)
  , robot_(std::make_shared<ParamRobot>())
{
}

MoveGroupHelper::~MoveGroupHelper()
{
    goal_sub_.shutdown();
    result_sub_.shutdown();
    gpsc_.shutdown();
    apsc_.shutdown();
}

void MoveGroupHelper::setResultCallback(const ResultCallback &callback)
{
    callback_ = callback;
}

bool MoveGroupHelper::executeTrajectory(const robot_trajectory::RobotTrajectory &path)
{
    if (!eac_.isServerConnected())
        return false;

    moveit_msgs::ExecuteTrajectoryGoal goal;
    path.getRobotTrajectoryMsg(goal.trajectory);

    eac_.sendGoal(goal);
    if (!eac_.waitForResult())
        ROS_INFO("ExecuteTrajectory action returned early");

    if (eac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;
    else
        return false;
}

bool MoveGroupHelper::pullState(RobotPtr &robot)
{
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE;

    if (gpsc_.call(request, response))
    {
        robot->setState(response.scene.robot_state);
        return true;
    }

    return false;
}

bool MoveGroupHelper::pullScene(ScenePtr &scene)
{
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS |
                                    moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
                                    moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                    moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                    moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                    moveit_msgs::PlanningSceneComponents::OCTOMAP |
                                    moveit_msgs::PlanningSceneComponents::TRANSFORMS |
                                    moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                    moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
                                    moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;

    if (gpsc_.call(request, response))
    {
        scene->useMessage(response.scene);
        return true;
    }

    return false;
}

bool MoveGroupHelper::pushScene(const SceneConstPtr &scene)
{
    moveit_msgs::ApplyPlanningScene::Request request;
    moveit_msgs::ApplyPlanningScene::Response response;
    request.scene = scene->getMessage();

    if (gpsc_.call(request, response))
        return true;

    return false;
}

void MoveGroupHelper::moveGroupGoalCallback(const moveit_msgs::MoveGroupActionGoal &msg)
{
    const std::string &id = msg.goal_id.id;
    ROS_DEBUG("Intercepted request goal ID: `%s`", id.c_str());

    Action action;
    action.scene.reset(new Scene(robot_));
    pullScene(action.scene);
    action.scene->useMessage(msg.goal.planning_options.planning_scene_diff);
    action.request = msg.goal.request;

    requests_.emplace(id, action);
}

void MoveGroupHelper::moveGroupResultCallback(const moveit_msgs::MoveGroupActionResult &msg)
{
    const std::string &id = msg.status.goal_id.id;

    auto request = requests_.find(id);
    if (request == requests_.end())
    {
        ROS_WARN("Intercepted unknown request response ID: `%s`", id.c_str());
        return;
    }

    ROS_DEBUG("Intercepted request response ID: `%s`", id.c_str());

    Action &action = request->second;
    action.id = id;
    action.success = msg.result.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
    action.time = msg.result.planning_time;
    action.trajectory = msg.result.planned_trajectory;

    if (callback_)
        callback_(action);

    requests_.erase(request);
}
