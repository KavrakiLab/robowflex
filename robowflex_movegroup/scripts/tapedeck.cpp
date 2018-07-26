/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_movegroup/services.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);
    movegroup::MoveGroupHelper helper;
    // helper.addResultCallback();

    ros::waitForShutdown();
}
