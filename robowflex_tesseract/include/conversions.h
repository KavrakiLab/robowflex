/* Author: Bryce Willey */

#ifndef ROBOWFLEX_TESSERACT_CONVERSIONS_
#define ROBOWFLEX_TESSERACT_CONVERSIONS_

#include <robowflex.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <moveit_msgs/PlanningScene.h>


namespace robowflex
{
    namespace robow_tesseract
    {
        tesseract::tesseract_ros::KDLEnvPtr constructTesseractEnv(moveit_msgs::PlanningScene scene_msg, robowflex::RobotConstPtr robot);

    }
}

#endif