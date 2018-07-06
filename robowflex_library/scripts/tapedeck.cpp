/* Author: Zachary Kingston */

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <robowflex_library/util.h>
#include <robowflex_library/io.h>
#include <robowflex_library/yaml.h>

using namespace robowflex;

class Tapedeck
{
public:
    Tapedeck(const std::string &move_group = "/move_group") : nh_("/")
    {
        psc_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene", true);
        nh_.subscribe(move_group + "/goal", 1, &Tapedeck::goalCallback, this);
        nh_.subscribe(move_group + "/result", 1, &Tapedeck::resultCallback, this);
    }

    moveit_msgs::PlanningScene retrieveScene()
    {
        moveit_msgs::GetPlanningScene::Request req;
        moveit_msgs::GetPlanningScene::Response res;

        req.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE           //
                                    | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES  //
                                    | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

        psc_.call(req, res);
        return res.scene;
    }

    void goalCallback(const moveit_msgs::MoveGroupActionGoal &msg)
    {
        const std::string &id = msg.goal_id.id;

        requests_.emplace(std::piecewise_construct,  //
                          std::forward_as_tuple(id),
                          std::forward_as_tuple(retrieveScene(), msg.goal.request));

        ROS_INFO("Intercepted request goal ID: `%s`", id.c_str());
    }

    void resultCallback(const moveit_msgs::MoveGroupActionResult &msg)
    {
        const std::string &id = msg.status.goal_id.id;

        auto request = requests_.find(id);
        if (request == requests_.end())
        {
            ROS_WARN("Intercepted unknown request response ID: `%s`", id.c_str());
            return;
        }

        ROS_WARN("Intercepted request response ID: `%s`", id.c_str());

        YAML::Node node;

        if (msg.result.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            node["success"] = "true";
            node["trajectory"] = msg.result.planned_trajectory;
        }
        else
            node["success"] = "false";

        node["id"] = id;
        node["time"] = ros::Time::now();
        node["scene"] = request->second.first;
        node["request"] = request->second.second;

        IO::YAMLtoFile(node, id + ".yml");
        ROS_WARN("  Wrote YAML for ID: `%s`", id.c_str());

        requests_.erase(request);
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient psc_;

    std::map<std::string, std::pair<moveit_msgs::PlanningScene, moveit_msgs::MotionPlanRequest>> requests_;
};

int main(int argc, char **argv)
{
    // Startup ROS
    startROS(argc, argv);
    Tapedeck deck;
    ros::spin();
}
