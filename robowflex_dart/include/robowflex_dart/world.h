/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_WORLD_
#define ROBOWFLEX_DART_WORLD_

#include <mutex>

#include <dart/dynamics/Skeleton.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/DistanceFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/simulation/World.hpp>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    namespace darts
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(Robot)
        ROBOWFLEX_CLASS_FORWARD(Structure)
        /** \endcond */

        /** \brief A wrapper for the ACM to pass into signed distance computation.
         *  Returns based on if the objects would collide.
         */
        class DistanceCollisionWrapper : public dart::collision::DistanceFilter
        {
        public:
            /** \brief Constructor.
             *  \param[in] filter Collision filter to wrap.
             */
            DistanceCollisionWrapper(const std::shared_ptr<dart::collision::CollisionFilter> &filter);

            bool needDistance(const dart::collision::CollisionObject *object1,
                              const dart::collision::CollisionObject *object2) const override;

        private:
            std::shared_ptr<dart::collision::CollisionFilter> filter_;  ///< Wrapped collision filter.
        };

        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(World)
        /** \endcond */

        /** \class robowflex::darts::WorldPtr
            \brief A shared pointer wrapper for robowflex::darts::World. */

        /** \class robowflex::darts::WorldConstPtr
            \brief A const shared pointer wrapper for robowflex::darts::World. */

        /** \brief Wrapper for a dart::simulation::World, which contains a set of skeletons (i.e., Robots and
         * Structures). The World is the main object that is used by motion planning as it contains a holistic
         * view of the scene.
         */
        class World
        {
            ROBOWFLEX_EIGEN

        public:
            /** \name Constuctors
                \{ */

            /** \brief Create an empty world.
             *  \param[in] name Name for the world.
             */
            World(const std::string &name = "world");

            /** \brief Clone this world.
             *  \param[in] suffix Suffix to append to all names in the world (i.e., robots).
             */
            WorldPtr clone(const std::string &suffix = "") const;

            /** \} */

            /** \name World Entity Management
                \{ */

            /** \brief Add a robot to the world.
             *  \param[in] robot Robot to add.
             */
            void addRobot(RobotPtr robot);

            /** \brief Remove a robot from the world.
             *  \param[in] name Name of robot to remove.
             */
            void removeRobot(const std::string &name);

            /** \brief Remove a robot from the world.
             *  \param[in] robot Robot to remove.
             */
            void removeRobot(const RobotPtr &robot);

            /** \brief Get a robot in the world.
             *  \param[in] name Name of robot to get.
             *  \return The robot, or nullptr if non-existent.
             */
            RobotPtr getRobot(const std::string &name);

            /** \brief Get a robot in the world.
             *  \param[in] name Name of robot to get.
             *  \return The robot, or nullptr if non-existent.
             */
            RobotConstPtr getRobotConst(const std::string &name) const;

            /** \brief Add a structure to the world.
             *  \param[in] structure Structure to add.
             */
            void addStructure(StructurePtr structure);

            /** \brief Remove a structure from the world.
             *  \param[in] name Name of structure to remove.
             */
            void removeStructure(const std::string &name);

            /** \brief Remove a structure from the world.
             *  \param[in] structure Structure to remove.
             */
            void removeStructure(const StructurePtr &structure);

            /** \brief Get a structure in the world.
             *  \param[in] name Name of structure to get.
             *  \return The structure, or nullptr if non-existent.
             */
            StructurePtr getStructure(const std::string &name);

            /** \brief Get a structure in the world.
             *  \param[in] name Name of structure to get.
             *  \return The structure, or nullptr if non-existent.
             */
            StructureConstPtr getStructureConst(const std::string &name) const;

            /** \brief Get the set of structures in the world.
             *  \return A constant reference to the set of structures.
             */
            const std::map<std::string, StructurePtr> &getStructures();

            /** \} */

            /** \name Getters and Setters
                \{ */

            /** \brief Get the name of this world.
             *  \return The name of the world.
             */
            const std::string &getName() const;

            /** \brief Get the bounds on the workspace.
             *  \return A pair of the lower and upper bounds on the workspace.
             */
            std::pair<Eigen::Vector3d, Eigen::Vector3d> getWorkspaceBounds() const;

            /** \brief Get the lower bounds of the workspace.
             *  \return A 3D vector of the lower X-, Y-, and Z-bounds on the workspace.
             */
            Eigen::Vector3d &getWorkspaceLow();

            /** \brief Get the lower bounds of the workspace.
             *  \return A 3D vector of the lower X-, Y-, and Z-bounds on the workspace.
             */
            const Eigen::Vector3d &getWorkspaceLowConst() const;

            /** \brief Get the upper bounds of the workspace.
             *  \return A 3D vector of the upper X-, Y-, and Z-bounds on the workspace.
             */
            Eigen::Vector3d &getWorkspaceHigh();

            /** \brief Get the upper bounds of the workspace.
             *  \return A 3D vector of the upper X-, Y-, and Z-bounds on the workspace.
             */
            const Eigen::Vector3d &getWorkspaceHighConst() const;

            /** \brief Get the Dart world.
             *  \return The underlying Dart world.
             */
            dart::simulation::WorldPtr getSim();

            /** \brief Get the Dart world.
             *  \return The underlying Dart world.
             */
            const dart::simulation::WorldPtr &getSimConst() const;

            /** \brief Get the index in the world of a skeleton.
             *  \param[in] skeleton The skeleton to get the index of.
             *  \return The index of a skeleton in the world.
             */
            unsigned int getSkeletonIndex(const dart::dynamics::SkeletonPtr &skeleton) const;

            /** \} */

            /** \name Collisions
                \{ */

            /** \brief Get a collision filter that allows collisions between all body nodes. Useful for
             *  constructing a custom filter to select one or a few objects.
             *  \return A collision filter that considers no collisions.
             */
            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> getAllValidFilter() const;

            /** \brief Get a collision filter that is a copy of the default filter.
             *  \return A copy of the default collision filter.
             */
            std::shared_ptr<dart::collision::BodyNodeCollisionFilter> getDefaultFilter() const;

            /** \brief Checks if world is currently in collision.
             *  \param[in] filter Custom collision filter to use. If null, will use default collision filter.
             *  \return True if the world is currently in collision, false otherwise.
             */
            bool inCollision(const std::shared_ptr<dart::collision::CollisionFilter> &filter = nullptr) const;

            /** \brief Gets the current signed distance to collision in the world.
             *  \return The signed distance to collision for the current state of the world.
             */
            double distanceToCollision() const;

            /** \brief Force forward kinematics to update.
             */
            void forceUpdate();

            /** \} */

            /** \brief Clear all IK modules created in skeletons in the world.
             */
            void clearIKModules();

            /** \brief Grab internal recursive lock for world.
             */
            void lock();

            /** \brief Unlock internal recursive world lock.
             */
            void unlock();

            /** \brief Open the Open Scene Graph visualization for this world.
             */
            void openOSGViewer();

            /** \brief Get the collision detector for this world.
             *  \return A pointer to the world's collision detector.
             */
            dart::collision::CollisionDetectorPtr getCollider() const;

            /** \brief Get the self-collision group for a skeleton in this world.
             *  \param[in] name The name of the skeleton.
             *  \return A pointer to the self-collision group for the skeleton.
             */
            std::shared_ptr<dart::collision::CollisionGroup>
            getSelfCollisionGroup(const std::string &name) const;

            /** \brief Get the group for everything other than a given skeleton in this world.
             *  \param[in] name The name of the skeleton.
             *  \return A pointer to the other-collision group for the skeleton.
             */
            std::shared_ptr<dart::collision::CollisionGroup>
            getOtherCollisionGroup(const std::string &name) const;

            /** \brief Collision filter information.
             */
            struct CollisionInfo
            {
                std::shared_ptr<dart::collision::CollisionGroup> self;    ///< All nodes belonging to a
                                                                          ///< skeleton.
                std::shared_ptr<dart::collision::CollisionGroup> others;  ///< All other nodes in world.
            };

            /** \brief Get the collision info (self and other collision groups) for a skeleton in this
             * world.
             * \param[in] name The name of the skeleton.
             * \return A struct containing pointers to the skeleton's self and other collision groups.
             */
            CollisionInfo getCollisionInfo(const std::string &name) const;

            /** \brief Get the current world collision filter (composite of all skeleton filters).
             * This is more efficient than constructing a new filter from
             * robowflex::darts::World::getDefaultFilter() or robowflex::darts::World::getAllValidFilter().
             * \return A pointer to the current world collision filter.
             */
            std::shared_ptr<dart::collision::CompositeCollisionFilter> &getWorldCollisionFilter();

        private:
            /** \brief Add a new collision filter (ACM) for a skeleton.
             *  \param[in] name Name for collision filter.
             *  \param[in] skeleton Skeleton collision filter is for.
             */
            void addSkeletonCollider(const std::string &name, const dart::dynamics::SkeletonPtr &skeleton);

            /** \brief Remove a collision filter (ACM).
             *  \param[in] name Name for collision filter.
             *  \param[in] skeleton Skeleton collision filter is for.
             */
            void removeSkeletonCollider(const std::string &name, const dart::dynamics::SkeletonPtr &skeleton);

            dart::simulation::WorldPtr world_;  ///< Underlying world.

            Eigen::Vector3d low_{-5, -5, -5};  ///< Lower workspace bounds.
            Eigen::Vector3d high_{5, 5, 5};    ///< Upper workspace bounds.

            std::map<std::string, RobotPtr> robots_;          ///< Robots in world.
            std::map<std::string, StructurePtr> structures_;  ///< Structures in world.

            dart::collision::CollisionGroupPtr all_;          ///< All collision groups in world.
            std::map<std::string, CollisionInfo> collision_;  ///< Map of skeleton names to collision
                                                              ///< filters.

            std::shared_ptr<dart::collision::CompositeCollisionFilter> filter_;  ///< Composite collision
                                                                                 ///< filter of skeleton
                                                                                 ///< filters.
            dart::collision::CollisionDetectorPtr collider_;                     ///< Collision checker.
            const std::string name_;                                             ///< Name of world.

            std::recursive_mutex mutex_;  ///< Internal lock.
        };
    }  // namespace darts
}  // namespace robowflex

#endif
