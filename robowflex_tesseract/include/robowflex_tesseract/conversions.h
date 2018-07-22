/* Author: Bryce Willey */

#ifndef ROBOWFLEX_TESSERACT_CONVERSIONS_
#define ROBOWFLEX_TESSERACT_CONVERSIONS_

#include <robowflex_library/robowflex.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <moveit_msgs/PlanningScene.h>


namespace robowflex
{
    namespace robow_tesseract
    {
        tesseract::tesseract_ros::KDLEnvPtr constructTesseractEnv(robowflex::SceneConstPtr scene, robowflex::RobotConstPtr robot);
    }
}

#endif