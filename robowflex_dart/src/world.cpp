/* Author: Zachary Kingston */

#include <dart/collision/CollisionObject.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/gui/osg/osg.hpp>

#include <robowflex_dart/acm.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>

using namespace robowflex::darts;

///
/// DistanceCollisionWrapper
///

DistanceCollisionWrapper::DistanceCollisionWrapper(
    const std::shared_ptr<dart::collision::CollisionFilter> &filter)
  : filter_(filter)
{
}

bool DistanceCollisionWrapper::needDistance(const dart::collision::CollisionObject *object1,
                                            const dart::collision::CollisionObject *object2) const
{
    return not filter_->ignoresCollision(object1, object2);

    // auto shapeNode1 = object1->getShapeFrame()->asShapeNode();
    // auto shapeNode2 = object2->getShapeFrame()->asShapeNode();

    // auto bodyNode1 = shapeNode1->getBodyNodePtr();
    // auto bodyNode2 = shapeNode2->getBodyNodePtr();

    // std::cout << r                                                                            //
    //           << ", " << object1->getShapeFrame()->getName() << ", " << bodyNode1->getName()  //
    //           << ", " << object2->getShapeFrame()->getName() << ", " << bodyNode2->getName() << std::endl;
}

///
/// World
///

World::World()
  : world_(dart::simulation::World::create())
  , filter_(std::make_shared<dart::collision::CompositeCollisionFilter>())
{
    world_->setGravity(Eigen::Vector3d(0, 0, -9.81));

    auto solver = world_->getConstraintSolver();
    auto &opt = solver->getCollisionOption();
    opt.collisionFilter = filter_;

    collider_ = solver->getCollisionDetector();
    all_ = collider_->createCollisionGroupAsSharedPtr();
}

void World::addSkeletonCollider(const std::string &name, const dart::dynamics::SkeletonPtr &skeleton)
{
    // Collect collision groups
    CollisionInfo info;
    info.self = collider_->createCollisionGroupAsSharedPtr();
    info.self->addShapeFramesOf(skeleton.get());
    all_->addShapeFramesOf(skeleton.get());

    info.others = collider_->createCollisionGroupAsSharedPtr();
    for (const auto &entry : collision_)
        info.others->addShapeFramesOf(entry.second.self.get());

    collision_.emplace(name, info);
}

void World::removeSkeletonCollider(const std::string &name, const dart::dynamics::SkeletonPtr &skeleton)
{
    // Remove from collision groups
    auto ic = collision_.find(name);
    collision_.erase(ic);

    for (auto &entry : collision_)
        entry.second.others->removeShapeFramesOf(skeleton.get());

    all_->removeShapeFramesOf(skeleton.get());
}

void World::addRobot(RobotPtr robot)
{
    auto it = robots_.find(robot->getName());
    if (it == robots_.end())
    {
        // Add robot to world
        robots_.emplace(robot->getName(), robot);
        world_->addSkeleton(robot->getSkeleton());

        // Add ACM to collision filter
        filter_->addCollisionFilter(robot->getACM()->getFilter().get());
        addSkeletonCollider(robot->getName(), robot->getSkeletonConst());
    }
}

void World::removeRobot(const std::string &name)
{
    auto it = robots_.find(name);
    if (it != robots_.end())
    {
        // Remove robot from world
        auto robot = it->second;
        world_->removeSkeleton(robot->getSkeleton());

        // Remove ACM from filter
        filter_->removeCollisionFilter(robot->getACM()->getFilter().get());
        removeSkeletonCollider(robot->getName(), robot->getSkeletonConst());

        robots_.erase(it);
    }
}

void World::removeRobot(RobotPtr robot)
{
    removeRobot(robot->getName());
}

RobotPtr World::getRobot(const std::string &name)
{
    return robots_.find(name)->second;
}

void World::addStructure(StructurePtr structure)
{
    auto it = structures_.find(structure->getName());
    if (it == structures_.end())
    {
        structures_.emplace(structure->getName(), structure);
        world_->addSkeleton(structure->getSkeleton());

        filter_->addCollisionFilter(structure->getACM()->getFilter().get());
    }
}

void World::removeStructure(const std::string &name)
{
    auto it = structures_.find(name);
    if (it != structures_.end())
    {
        auto structure = it->second;
        world_->removeSkeleton(structure->getSkeleton());
        filter_->removeCollisionFilter(structure->getACM()->getFilter().get());
        structures_.erase(it);
    }
}

void World::removeStructure(StructurePtr structure)
{
    removeStructure(structure->getName());
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> World::getWorkspaceBounds() const
{
    return std::make_pair(low_, high_);
}

Eigen::Vector3d &World::getWorkspaceLow()
{
    return low_;
}

const Eigen::Vector3d &World::getWorkspaceLowConst() const
{
    return low_;
}

Eigen::Vector3d &World::getWorkspaceHigh()
{
    return high_;
}

const Eigen::Vector3d &World::getWorkspaceHighConst() const
{
    return high_;
}

dart::simulation::WorldPtr World::getSim()
{
    return world_;
}

const dart::simulation::WorldPtr &World::getSimConst() const
{
    return world_;
}

unsigned int World::getSkeletonIndex(dart::dynamics::SkeletonPtr skeleton) const
{
    for (unsigned int i = 0; i < world_->getNumSkeletons(); ++i)
    {
        if (skeleton == world_->getSkeleton(i))
            return i;
    }

    return -1;
}

bool World::inCollision() const
{
    dart::collision::CollisionOption option;
    option.collisionFilter = filter_;

    return collider_->collide(all_.get(), option, nullptr);
}

double World::distanceToCollision() const
{
    dart::collision::DistanceOption option;
    option.distanceLowerBound = -std::numeric_limits<double>::infinity();
    option.distanceFilter = std::make_shared<DistanceCollisionWrapper>(filter_);

    dart::collision::DistanceResult result;
    double d = collider_->distance(all_.get(), option, &result);

    const auto &shapeNode1 = result.shapeFrame1->asShapeNode();
    const auto &shapeNode2 = result.shapeFrame2->asShapeNode();

    const auto &bodyNode1 = shapeNode1->getBodyNodePtr();
    const auto &bodyNode2 = shapeNode2->getBodyNodePtr();

    std::cout << d                                                                            //
              << ", " << shapeNode1->getName() << ", " << bodyNode1->getName()  //
              << ", " << shapeNode2->getName() << ", " << bodyNode2->getName() << std::endl;

    return d;
}

void World::openOSGViewer()
{
    ::osg::ref_ptr<dart::gui::osg::RealTimeWorldNode> node = new dart::gui::osg::RealTimeWorldNode(world_);

    auto viewer = dart::gui::osg::Viewer();
    viewer.addWorldNode(node);

    viewer.setUpViewInWindow(0, 0, 1080, 1080);
    viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3(2.57, 3.14, 1.64), ::osg::Vec3(0.00, 0.00, 0.00), ::osg::Vec3(-0.24, -0.25, 0.94));
    viewer.setCameraManipulator(viewer.getCameraManipulator());
    viewer.run();
}
