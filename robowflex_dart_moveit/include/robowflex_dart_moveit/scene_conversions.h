/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_MOVEIT_STRUCTURE_
#define ROBOWFLEX_DART_MOVEIT_STRUCTURE_

#include <robowflex_moveit/core/scene.h>

#include <robowflex_dart/structure.h>

namespace robowflex::darts::conversions
{
    /** \brief Copy a MoveIt (robowflex::Scene) into a structure.
     *  \param[in] name Name of the structure.
     *  \param[in] scene Scene to copy.
     */
    darts::StructurePtr fromMoveItScene(const std::string &name, const robowflex::SceneConstPtr &scene);
}  // namespace robowflex::darts::conversions

#endif
