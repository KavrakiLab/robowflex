/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ADAPTER_
#define ROBOWFLEX_ADAPTER_

#include <type_traits>

#include <moveit/transforms/transforms.h>

namespace robowflex
{
    using RobotPose = std::remove_cv<                                      //
        std::remove_reference<                                             //
            decltype(                                                      //
                std::declval<moveit::core::Transforms>().getTransform("")  //
                )                                                          //
            >::type                                                        //
        >::type;
}  // namespace robowflex

#endif
