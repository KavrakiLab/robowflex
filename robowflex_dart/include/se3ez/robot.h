/* Author: Zachary Kingston */

#ifndef SE3EZ_ROBOT_
#define SE3EZ_ROBOT_

#include <dart/dynamics/Skeleton.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/simulation/World.hpp>

#include <se3ez/util.h>
#include <se3ez/io.h>

namespace se3ez
{
    SE3EZ_CLASS(Structure)
    SE3EZ_CLASS(Robot)
    SE3EZ_CLASS(Environment)
    SE3EZ_CLASS(World)

    class ACM
    {
    public:
        ACM(const Structure &robot);

        void disableCollision(const std::string &a, const std::string &b);
        void enableCollision(const std::string &a, const std::string &b);

        std::shared_ptr<dart::collision::BodyNodeCollisionFilter> getFilter();
        const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &getFilterConst() const;

        const Structure &getStructure() const;

        std::set<std::pair<std::string, std::string>> &getDisabledPairs();
        const std::set<std::pair<std::string, std::string>> &getDisabledPairsConst() const;

    private:
        std::pair<std::string, std::string> makeKey(const std::string &a, const std::string &b) const;
        dart::dynamics::BodyNode *getBodyNode(const std::string &key);

        const Structure &structure_;
        std::set<std::pair<std::string, std::string>> acm_;
        std::shared_ptr<dart::collision::BodyNodeCollisionFilter> filter_;
    };

    class Structure
    {
    public:
        struct Frame
        {
            dart::dynamics::BodyNode *node;
            dart::dynamics::Joint *joint;
        };

        Structure(const std::string &name);

        const std::string &getName() const;

        ACM &getACM();
        const ACM &getACMConst() const;

        void setSkeleton(const dart::dynamics::SkeletonPtr &skeleton);
        dart::dynamics::SkeletonPtr &getSkeleton();
        const dart::dynamics::SkeletonPtr &getSkeletonConst() const;

    protected:
        const std::string name_{"robot"};
        dart::dynamics::SkeletonPtr skeleton_{nullptr};
        ACM acm_;
    };

    class Robot : public Structure
    {
    public:
        Robot(const std::string &name);

        RobotPtr clone(const std::string &newName) const;

        void loadURDF(const std::string &urdf);
        void loadSRDF(const std::string &srdf);

        const std::string &getname() const;

        bool isGroup(const std::string &name) const;
        std::map<std::string, std::vector<std::string>> &getGroups();
        const std::map<std::string, std::vector<std::string>> &getGroupsConst();
        std::vector<std::string> getGroupJointNames(const std::string &name) const;
        std::vector<dart::dynamics::Joint *> getGroupJoints(const std::string &name) const;

        void setDof(unsigned int index, double value);

    private:
        std::map<std::string, std::vector<std::string>> groups_;
    };

    RobotPtr loadMoveItRobot(const std::string &name, const std::string &urdf, const std::string &srdf);

    class Environment : public Structure
    {
    public:
        Environment(const std::string &name);

        void addGround();
    };

    class World
    {
        SE3EZ_EIGEN

    public:
        World();

        void addRobot(RobotPtr robot);
        void removeRobot(const std::string &name);
        void removeRobot(RobotPtr robot);
        RobotPtr getRobot(const std::string &name);

        void addEnvironment(EnvironmentPtr environment);
        void removeEnvironment(const std::string &name);
        void removeEnvironment(EnvironmentPtr environment);

        std::pair<Eigen::Vector3d, Eigen::Vector3d> getWorkspaceBounds() const;
        Eigen::Vector3d &getWorkspaceLow();
        const Eigen::Vector3d &getWorkspaceLowConst() const;
        Eigen::Vector3d &getWorkspaceHigh();
        const Eigen::Vector3d &getWorkspaceHighConst() const;

        bool inCollision() const;

        void openOSGViewer();

    private:
        dart::simulation::WorldPtr world_;

        Eigen::Vector3d low_{-2, -2, -2};
        Eigen::Vector3d high_{2, 2, 2};

        std::shared_ptr<dart::collision::CompositeCollisionFilter> filter_;

        std::map<std::string, RobotPtr> robots_;
        std::map<std::string, EnvironmentPtr> environments_;
    };
}  // namespace se3ez

#endif
