/* Author: Bryce Willey */

#ifndef ROBOWFLEX_TESSERACT_CONVERSIONS_
#define ROBOWFLEX_TESSERACT_CONVERSIONS_

#include <robowflex_library/macros.h>

#include <tesseract_ros/kdl/kdl_env.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Robot);
    /** \endcond */

    namespace hypercube
    {
        tesseract::tesseract_ros::KDLEnvPtr constructTesseractEnv(const robowflex::SceneConstPtr &scene,
                                                                  const robowflex::RobotConstPtr &robot);
    }
}  // namespace robowflex

#endif
