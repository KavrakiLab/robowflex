/* Author: Zachary Kingston */

#ifndef SE3EZ_UTIL_
#define SE3EZ_UTIL_

#include <memory>
#include <Eigen/Core>

#define SE3EZ_CLASS(C)                                                                                       \
    class C;                                                                                                 \
    typedef std::shared_ptr<C> C##Ptr;                                                                       \
    typedef std::shared_ptr<const C> C##ConstPtr;

#define SE3EZ_EIGEN EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#endif
