/* Author: Zachary Kingston */

#include <boost/date_time.hpp>

#include <robowflex_library/log.h>
#include <robowflex_library/util.h>

#include <robowflex_movegroup/services.h>

using namespace robowflex;

/* \file tapedeck.cpp
 * An example script that shows how to use MoveGroupHelper. Here, a callback is
 * installed so that every motion plan issued to MoveGroup is saved to disk as a
 * YAML file. This is useful for collecting and replaying motion planning
 * requests that are done over the course of an experiment.
 */

// Output a captured action to a YAML file.
void callback(movegroup::MoveGroupHelper::Action &action)
{
    ros::Time time = ros::Time::now();

    // Save the requests to a folder in the home directory.
    const std::string filename = "~/robowflex_tapedeck/" + to_iso_string(time.toBoost()) + ".yml";
    action.toYAMLFile(filename);

    RBX_INFO("Wrote YAML for Request ID `%s` to file `%s`", action.id, filename);
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
