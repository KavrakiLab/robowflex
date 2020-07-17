/* Author: Bryce Willey */

#ifndef ROBOWFLEX_TESSERACT_CONVERSIONS_
#define ROBOWFLEX_TESSERACT_CONVERSIONS_

#include <robowflex_library/class_forward.h>

#include <tesseract_ros/kdl/kdl_env.h>

#include <tesseract_planning/basic_planner_types.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Robot);
    /** \endcond */

    namespace hypercube
    {
        tesseract::tesseract_ros::KDLEnvPtr constructTesseractEnv(const robowflex::SceneConstPtr &scene,
                                                                  const robowflex::RobotConstPtr &robot, 
                                                                  const std::string &name=""
                                                                 );
        void fromTesseractResToMoveitTraj(const tesseract::tesseract_planning::PlannerResponse &response, 
                                          const tesseract::tesseract_ros::KDLEnvPtr &env, 
                                          const robowflex::RobotConstPtr &robot, 
                                          robot_trajectory::RobotTrajectoryPtr &trajectory);
    }
}  // namespace robowflex

#endif
