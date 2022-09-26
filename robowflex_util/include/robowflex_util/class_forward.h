/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_CLASS_FORWARD_
#define ROBOWFLEX_CLASS_FORWARD_

#include <memory>  // for std::shared_ptr

/** \file */

/**
 * \def ROBOWFLEX_CLASS_FORWARD
 * \brief Macro that forward declares a class and defines two shared ptrs types:
 *  - ${Class}Ptr      = shared_ptr<${Class}>
 *  - ${Class}ConstPtr = shared_ptr<const ${Class}>
 */
#define ROBOWFLEX_CLASS_FORWARD(C)                                                                           \
    class C;                                                                                                 \
    typedef std::shared_ptr<C> C##Ptr;                                                                       \
    typedef std::shared_ptr<const C> C##ConstPtr;

/**
 * \def ROBOWFLEX_EIGEN
 * \brief Macro for classes with fixed width Eigen classes.
 */
#define ROBOWFLEX_EIGEN EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#endif
