/* Author: Zachary Kingston */

#include <tinyxml2.h>

#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

#include <dart/dynamics/WeldJoint.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/gui/osg/osg.hpp>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>

using namespace robowflex::darts;

///
/// ACM
///

ACM::ACM(const Structure &structure)
  : structure_(structure), filter_(std::make_shared<dart::collision::BodyNodeCollisionFilter>())
{
}

void ACM::disableCollision(const std::string &a, const std::string &b)
{
    auto key = makeKey(a, b);
    if (acm_.find(key) == acm_.end())
    {
        acm_.emplace(key);
        filter_->addBodyNodePairToBlackList(getBodyNode(key.first), getBodyNode(key.second));
    }
}

void ACM::enableCollision(const std::string &a, const std::string &b)
{
    auto key = makeKey(a, b);
    auto it = acm_.find(key);
    if (it != acm_.end())
    {
        filter_->removeBodyNodePairFromBlackList(getBodyNode(key.first), getBodyNode(key.second));
        acm_.erase(it);
    }
}

std::shared_ptr<dart::collision::BodyNodeCollisionFilter> ACM::getFilter()
{
    return filter_;
}

const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &ACM::getFilterConst() const
{
    return filter_;
}

const Structure &ACM::getStructure() const
{
    return structure_;
}

std::set<std::pair<std::string, std::string>> &ACM::getDisabledPairs()
{
    return acm_;
}

const std::set<std::pair<std::string, std::string>> &ACM::getDisabledPairsConst() const
{
    return acm_;
}

std::pair<std::string, std::string> ACM::makeKey(const std::string &a, const std::string &b) const
{
    if (a < b)
        return std::make_pair(a, b);
    else
        return std::make_pair(b, a);
}

dart::dynamics::BodyNode *ACM::getBodyNode(const std::string &key)
{
    return structure_.getSkeletonConst()->getBodyNode(key);
}

///
/// Structure
///

Structure::Structure(const std::string &name)
  : name_(name), skeleton_(dart::dynamics::Skeleton::create(name_)), acm_(*this)
{
}

const std::string &Structure::getName() const
{
    return name_;
}

ACM &Structure::getACM()
{
    return acm_;
}

const ACM &Structure::getACMConst() const
{
    return acm_;
}

void Structure::setSkeleton(const dart::dynamics::SkeletonPtr &skeleton)
{
    skeleton_ = skeleton;
    skeleton_->setName(name_);
}

dart::dynamics::SkeletonPtr &Structure::getSkeleton()
{
    return skeleton_;
}

const dart::dynamics::SkeletonPtr &Structure::getSkeletonConst() const
{
    return skeleton_;
}

///
/// Robot
///

Robot::Robot(const std::string &name) : Structure(name)
{
}

RobotPtr Robot::clone(const std::string &newName) const
{
    auto robot = std::make_shared<Robot>(newName);
    robot->setSkeleton(skeleton_->cloneSkeleton());

    for (const auto &pair : acm_.getDisabledPairsConst())
        robot->getACM().disableCollision(pair.first, pair.second);

    robot->getGroups() = groups_;

    return robot;
}

void Robot::loadURDF(const std::string &urdf)
{
    IO::loadURDF(*this, urdf);
}

void Robot::loadSRDF(const std::string &srdf)
{
    const auto &file = IO::getPackageFile(srdf);

    tinyxml2::XMLDocument doc;
    doc.LoadFile(file.c_str());

    tinyxml2::XMLElement *root = doc.FirstChildElement();

    // Parse groups
    tinyxml2::XMLElement *group = root->FirstChildElement("group");
    while (group != nullptr)
    {
        const auto &name = group->Attribute("name");
        // std::cout << "group: " << name << std::endl;

        std::vector<std::string> dofs;

        tinyxml2::XMLElement *links = group->FirstChildElement("link");
        while (links != nullptr)
        {
            const auto &name = links->Attribute("name");
            // std::cout << "link: " << name << std::endl;
            const auto &node = skeleton_->getBodyNode(name);
            const auto &joint = node->getParentJoint();

            if (joint->getNumDofs() > 0)
            {
                // std::cout << "  joint: " << joint->getName() << std::endl;
                dofs.emplace_back(joint->getName());
            }

            links = links->NextSiblingElement("link");
        }

        tinyxml2::XMLElement *joints = group->FirstChildElement("joint");
        while (joints != nullptr)
        {
            const auto &name = joints->Attribute("name");
            const auto &joint = skeleton_->getJoint(name);
            if (joint->getNumDofs() > 0)
            {
                // std::cout << "joint: " << joint->getName() << std::endl;
                dofs.emplace_back(joint->getName());
            }

            joints = joints->NextSiblingElement("joint");
        }

        tinyxml2::XMLElement *chains = group->FirstChildElement("chain");
        while (chains != nullptr)
        {
            const auto &base_link = chains->Attribute("base_link");
            const auto &tip_link = chains->Attribute("tip_link");
            // std::cout << "chain: "                                                                  //
            //           << chains->Attribute("base_link") << " -> " << chains->Attribute("tip_link")  //
            //           << std::endl;

            auto *node = skeleton_->getBodyNode(tip_link);
            while (node->getName() != base_link)
            {
                // std::cout << "  link: " << node->getName() << std::endl;
                const auto &joint = node->getParentJoint();

                if (joint->getNumDofs() > 0)
                {
                    // std::cout << "  joint: " << joint->getName() << std::endl;
                    dofs.emplace_back(joint->getName());
                }

                node = joint->getParentBodyNode();
            }

            chains = chains->NextSiblingElement("chain");
        }

        tinyxml2::XMLElement *groups = group->FirstChildElement("group");
        while (groups != nullptr)
        {
            const auto &name = groups->Attribute("name");

            auto it = groups_.find(name);
            if (it != groups_.end())
                dofs.insert(dofs.end(), it->second.begin(), it->second.end());

            groups = groups->NextSiblingElement("group");
        }

        std::cout << "group: " << name << std::endl;
        for (const auto &dof : dofs)
        {
            std::cout << "  " << dof << ": " << skeleton_->getJoint(dof)->getType() << std::endl;
        }
        groups_.emplace(name, dofs);

        group = group->NextSiblingElement("group");
    }

    // Parse allowed collisions
    tinyxml2::XMLElement *dc = root->FirstChildElement("disable_collisions");
    while (dc != nullptr)
    {
        const auto &l1 = dc->Attribute("link1");
        const auto &l2 = dc->Attribute("link2");

        acm_.disableCollision(l1, l2);

        // auto a = skeleton_->getBodyNode(el->Attribute("link1"));
        // auto b = skeleton_->getBodyNode(el->Attribute("link2"));

        dc = dc->NextSiblingElement("disable_collisions");
    }
}

bool Robot::isGroup(const std::string &name) const
{
    return groups_.find(name) != groups_.end();
}

std::map<std::string, std::vector<std::string>> &Robot::getGroups()
{
    return groups_;
}

const std::map<std::string, std::vector<std::string>> &Robot::getGroupsConst()
{
    return groups_;
}

std::vector<std::string> Robot::getGroupJointNames(const std::string &name) const
{
    // Need to setup error reporting
    if (!isGroup(name))
    {
    }

    return groups_.find(name)->second;
}

std::vector<dart::dynamics::Joint *> Robot::getGroupJoints(const std::string &name) const
{
    // Need to setup error reporting
    if (!isGroup(name))
    {
    }

    std::vector<dart::dynamics::Joint *> joints;
    for (const auto &dof : groups_.find(name)->second)
        joints.emplace_back(skeleton_->getJoint(dof));

    return joints;
}

void Robot::setDof(unsigned int index, double value)
{
    skeleton_->getDof(index)->setPosition(value);
}

RobotPtr robowflex::darts::loadMoveItRobot(const std::string &name, const std::string &urdf,
                                           const std::string &srdf)
{
    auto robot = std::make_shared<Robot>(name);
    robot->loadURDF(urdf);
    robot->loadSRDF(srdf);

    return robot;
}

///
/// Environment
///

Environment::Environment(const std::string &name) : Structure(name)
{
}

void Environment::addGround()
{
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    double thickness = 0.01;
    tf.translation() = Eigen::Vector3d(0, 0, -thickness / 2.0);
    dart::dynamics::WeldJoint::Properties joint;
    joint.mT_ParentBodyToJoint = tf;
    skeleton_->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr, joint);
    dart::dynamics::ShapePtr groundShape =
        std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(10, 10, thickness));

    auto shapeNode = skeleton_->getBodyNode(0)
                         ->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect,
                                               dart::dynamics::DynamicsAspect>(groundShape);

    shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));
}

///
/// World
///

World::World()
  : world_(dart::simulation::World::create())
  , filter_(std::make_shared<dart::collision::CompositeCollisionFilter>())
{
    world_->setGravity(Eigen::Vector3d(0, 0, -9.81));

    auto &opt = world_->getConstraintSolver()->getCollisionOption();
    opt.collisionFilter = filter_;
}

void World::addRobot(RobotPtr robot)
{
    auto it = robots_.find(robot->getName());
    if (it == robots_.end())
    {
        robots_.emplace(robot->getName(), robot);
        world_->addSkeleton(robot->getSkeleton());

        filter_->addCollisionFilter(robot->getACM().getFilter().get());
    }
}

void World::removeRobot(const std::string &name)
{
    auto it = robots_.find(name);
    if (it != robots_.end())
    {
        auto robot = it->second;
        world_->removeSkeleton(robot->getSkeleton());
        filter_->removeCollisionFilter(robot->getACM().getFilter().get());
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

void World::addEnvironment(EnvironmentPtr environment)
{
    auto it = environments_.find(environment->getName());
    if (it == environments_.end())
    {
        environments_.emplace(environment->getName(), environment);
        world_->addSkeleton(environment->getSkeleton());

        filter_->addCollisionFilter(environment->getACM().getFilter().get());
    }
}

void World::removeEnvironment(const std::string &name)
{
    auto it = environments_.find(name);
    if (it != environments_.end())
    {
        auto environment = it->second;
        world_->removeSkeleton(environment->getSkeleton());
        filter_->removeCollisionFilter(environment->getACM().getFilter().get());
        environments_.erase(it);
    }
}

void World::removeEnvironment(EnvironmentPtr environment)
{
    removeEnvironment(environment->getName());
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

bool World::inCollision() const
{
    dart::collision::CollisionOption option;
    option.collisionFilter = filter_;
    dart::collision::CollisionResult result;
    // bool r = world_->checkCollision(option, &result);
    bool r = world_->checkCollision(option, nullptr);

    // for (const auto &body : result.getCollidingShapeFrames())
    // {
    //     std::cout << body->getName() <<  std::endl;
    // }

    return r;
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
