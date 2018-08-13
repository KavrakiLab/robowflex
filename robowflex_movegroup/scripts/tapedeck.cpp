/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>

#include <robowflex_movegroup/services.h>

using namespace robowflex;

// Output a captured action to a YAML file.
void callback(movegroup::MoveGroupHelper::Action &action)
{
    YAML::Node node;

    node["id"] = action.id;

    moveit_msgs::PlanningScene scene_msg;
    action.scene->getSceneConst()->getPlanningSceneMsg(scene_msg);
    node["scene"] = IO::toNode(scene_msg);

    node["request"] = IO::toNode(action.request);
    node["success"] = action.success ? "true" : "false";
    node["time"] = action.time;
    node["trajectory"] = IO::toNode(action.trajectory);

    ros::Time time = ros::Time::now();
    IO::YAMLToFile(node, "~/robowflex_tapedeck/" + to_iso_string(time.toBoost()) + ".yml");
    ROS_INFO("Wrote YAML for Request ID: `%s`", action.id.c_str());
}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create helper
    movegroup::MoveGroupHelper helper;

    // Setup callback function
    helper.setResultCallback(callback);

    // Wait until killed
    ros.wait();
}
