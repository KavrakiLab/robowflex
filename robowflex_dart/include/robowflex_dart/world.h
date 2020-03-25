/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_WORLD_
#define ROBOWFLEX_DART_WORLD_

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
        ROBOWFLEX_CLASS_FORWARD(Robot)
        ROBOWFLEX_CLASS_FORWARD(Structure)
        ROBOWFLEX_CLASS_FORWARD(World)

        class DistanceCollisionWrapper : public dart::collision::DistanceFilter
        {
        public:
            DistanceCollisionWrapper(const std::shared_ptr<dart::collision::CollisionFilter> &filter);

            bool needDistance(const dart::collision::CollisionObject *object1,
                              const dart::collision::CollisionObject *object2) const override;

        private:
            std::shared_ptr<dart::collision::CollisionFilter> filter_;
        };

        class World
        {
            ROBOWFLEX_EIGEN

        public:
            World();

            void addRobot(RobotPtr robot);
            void removeRobot(const std::string &name);
            void removeRobot(RobotPtr robot);
            RobotPtr getRobot(const std::string &name);

            void addStructure(StructurePtr structure);
            void removeStructure(const std::string &name);
            void removeStructure(StructurePtr structure);

            std::pair<Eigen::Vector3d, Eigen::Vector3d> getWorkspaceBounds() const;
            Eigen::Vector3d &getWorkspaceLow();
            const Eigen::Vector3d &getWorkspaceLowConst() const;
            Eigen::Vector3d &getWorkspaceHigh();
            const Eigen::Vector3d &getWorkspaceHighConst() const;

            dart::simulation::WorldPtr getSim();
            const dart::simulation::WorldPtr &getSimConst() const;

            unsigned int getSkeletonIndex(dart::dynamics::SkeletonPtr skeleton) const;

            bool inCollision() const;
            double distanceToCollision() const;

            void openOSGViewer();

        private:
            void addSkeletonCollider(const std::string &name, const dart::dynamics::SkeletonPtr &skeleton);
            void removeSkeletonCollider(const std::string &name, const dart::dynamics::SkeletonPtr &skeleton);

            dart::simulation::WorldPtr world_;

            Eigen::Vector3d low_{-1, -1, -1};
            Eigen::Vector3d high_{1, 1, 1};

            std::map<std::string, RobotPtr> robots_;
            std::map<std::string, StructurePtr> structures_;

            struct CollisionInfo
            {
                std::shared_ptr<dart::collision::CollisionGroup> self;
                std::shared_ptr<dart::collision::CollisionGroup> others;
            };

            dart::collision::CollisionGroupPtr all_;
            std::map<std::string, CollisionInfo> collision_;

            std::shared_ptr<dart::collision::CompositeCollisionFilter> filter_;
            dart::collision::CollisionDetectorPtr collider_;
        };
    }  // namespace darts
}  // namespace robowflex

#endif
