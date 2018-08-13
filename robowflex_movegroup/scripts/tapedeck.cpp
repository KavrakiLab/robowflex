/* Author: Zachary Kingston */

#include <boost/date_time.hpp>

#include <robowflex_library/util.h>

#include <robowflex_movegroup/services.h>

using namespace robowflex;

// Output a captured action to a YAML file.
void callback(movegroup::MoveGroupHelper::Action &action)
{
    ros::Time time = ros::Time::now();
    const std::string filename = "~/robowflex_tapedeck/" + to_iso_string(time.toBoost()) + ".yml";
    action.toYAMLFile(filename);

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
