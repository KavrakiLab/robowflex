/* Author: Zachary Kingston */

#include <robowflex_movegroup/services.h>

const std::string MoveGroupHelper::MOVE_GROUP{"/move_group"};
const std::string MoveGroupHelper::GET_SCENE{"get_planning_scene"};
const std::string MoveGroupHelper::APPLY_SCENE{"apply_planning_scene"};
const std::string MoveGroupHelper::EXECUTE{"execute_trajectory"};

MoveGroupHelper::MoveGroupHelper(const std::string &move_group) : nh_("~")
{
    gpsc_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>(GET_SCENE, true);
    apsc_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>(APPLY_SCENE, true);

    eac_ = actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(
        nh_, EXECUTE, false);

    goal_sub_ = nh_.subscribe(move_group + "/goal", 10, &MoveGroupHelper::moveGroupGoalCallback, this);
    result_sub_ = nh_.subscribe(move_group + "/result", 10, &MoveGroupHelper::moveGroupResultCallback, this);
}

MoveGroupHelper::~MoveGroupHelper()
{
}

void MoveGroupHelper::setGoalCallback(const GoalCallback &callback)
{
    callback_ = callback;
}

void MoveGroupHelper::executeTrajectory(const std::string &group,
                                        const robot_trajectory::RobotTrajectory &path) const
{
}

void MoveGroupHelper::pullState(RobotPtr &robot) const
{
}

void MoveGroupHelper::pullScene(ScenePtr &scene) const
{
}

void MoveGroupHelper::pushScene(const SceneConstPtr &scene) const
{
}

void moveGroupGoalCallback(const moveit_msgs::MoveGroupActionGoal &msg)
{
    // const std::string &id = msg.goal_id.id;
    // ROS_INFO("Intercepted request goal ID: `%s`", id.c_str());

    // requests_.emplace(std::piecewise_construct,  //
    //                   std::forward_as_tuple(id), std::forward_as_tuple(retrieveScene(), msg.goal.request));
}

void moveGroupResultCallback(const moveit_msgs::MoveGroupActionResult &msg)
{
    // const std::string &id = msg.status.goal_id.id;

    // auto request = requests_.find(id);
    // if (request == requests_.end())
    // {
    //     ROS_WARN("Intercepted unknown request response ID: `%s`", id.c_str());
    //     return;
    // }

    // ROS_WARN("Intercepted request response ID: `%s`", id.c_str());

    // YAML::Node node;

    // if (msg.result.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    // {
    //     node["success"] = "true";
    //     node["trajectory"] = msg.result.planned_trajectory;
    // }
    // else
    //     node["success"] = "false";

    // ros::Time time = ros::Time::now();
    // node["id"] = id;
    // node["time"] = time;
    // node["scene"] = request->second.first;
    // node["request"] = request->second.second;

    // IO::YAMLToFile(node, "~/robowflex_tapedeck/" + to_iso_string(time.toBoost()) + ".yml");
    // ROS_WARN("  Wrote YAML for ID: `%s`", id.c_str());

    // requests_.erase(request);
}
