/* Author: Zachary Kingston */

#include <dart/collision/CollisionObject.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/gui/osg/osg.hpp>

#include <robowflex_library/log.h>

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
}

///
/// World
///

World::World(const std::string &name)
  : world_(dart::simulation::World::create(name))
  , filter_(std::make_shared<dart::collision::CompositeCollisionFilter>())
  , name_(name)
{
    world_->setGravity(Eigen::Vector3d(0, 0, -9.81));

    auto solver = world_->getConstraintSolver();

    auto &opt = solver->getCollisionOption();
    opt.collisionFilter = filter_;

    collider_ = solver->getCollisionDetector();

    // world_->getConstraintSolver()->setCollisionDetector(collider_->cloneWithoutCollisionObjects());

    all_ = collider_->createCollisionGroupAsSharedPtr();
}

WorldPtr World::clone(const std::string &suffix) const
{
    auto world = std::make_shared<World>(name_ + suffix);

    for (const auto &robot : robots_)
        world->addRobot(robot.second->cloneRobot(robot.first + suffix));

    for (const auto &structure : structures_)
        world->addStructure(structure.second->cloneStructure(structure.first + suffix));

    // for (std::size_t i = 0; i < world_->getNumSkeletons(); ++i)
    // {
    //     auto skel = world_->getSkeleton(i);
    //     auto state = skel->getState();
    //     auto other_skel = world->getSim()->getSkeleton(skel->getName() + suffix);
    //     other_skel->setState(state);
    // }

    return world;
}

const std::string &World::getName() const
{
    return name_;
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
    auto it = robots_.find(name);
    if (it != robots_.end())
        return it->second;

    return nullptr;
}

void World::addStructure(StructurePtr structure)
{
    auto it = structures_.find(structure->getName());
    if (it == structures_.end())
    {
        structures_.emplace(structure->getName(), structure);
        world_->addSkeleton(structure->getSkeleton());

        filter_->addCollisionFilter(structure->getACM()->getFilter().get());
        addSkeletonCollider(structure->getName(), structure->getSkeletonConst());
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
        removeSkeletonCollider(structure->getName(), structure->getSkeletonConst());

        structures_.erase(it);
    }
}

void World::removeStructure(StructurePtr structure)
{
    removeStructure(structure->getName());
}

StructurePtr World::getStructure(const std::string &name)
{
    auto it = structures_.find(name);
    if (it != structures_.end())
        return it->second;

    return nullptr;
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

    const auto &shape_node1 = result.shapeFrame1->asShapeNode();
    const auto &shape_node2 = result.shapeFrame2->asShapeNode();

    const auto &body_node1 = shape_node1->getBodyNodePtr();
    const auto &body_node2 = shape_node2->getBodyNodePtr();

    RBX_INFO("Distance B1:%s:%s -> B2:%s:%s = %d",           //
             shape_node1->getName(), body_node1->getName(),  //
             shape_node2->getName(), body_node2->getName(),  //
             d);

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

void World::forceUpdate()
{
    for (std::size_t i = 0; i < world_->getNumSkeletons(); ++i)
        world_->getSkeleton(i)->computeForwardKinematics();
}

void World::clearIKModules()
{
    for (std::size_t i = 0; i < world_->getNumSkeletons(); ++i)
    {
        auto skel = world_->getSkeleton(i);
        skel->clearIK();

        for (auto bn : skel->getBodyNodes())
            bn->clearIK();
    }
}

void World::lock()
{
    mutex_.lock();
}

void World::unlock()
{
    mutex_.unlock();
}
