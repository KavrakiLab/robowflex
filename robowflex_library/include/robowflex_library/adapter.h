/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ADAPTER_
#define ROBOWFLEX_ADAPTER_

#include <type_traits>

#include <moveit/transforms/transforms.h>

namespace robowflex
{
    /** \brief A pose (point in SE(3)) used in various functions. Defined from what \e MoveIt! uses. */
    using RobotPose = std::decay<                                      //
        decltype(                                                      //
            std::declval<moveit::core::Transforms>().getTransform("")  //
            )                                                          //
        >::type;
}  // namespace robowflex

#endif
