/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_ROBOT_
#define ROBOWFLEX_DART_ROBOT_

#include <dart/dynamics/Skeleton.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/simulation/World.hpp>

#include <robowflex_library/class_forward.h>

#include <robowflex_dart/io.h>
#include <robowflex_dart/structure.h>

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Robot)

    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(Robot)

        class Robot : public Structure
        {
        public:
            Robot(const std::string &name);
            Robot(robowflex::RobotPtr robot);

            RobotPtr clone(const std::string &newName) const;

            bool loadURDF(const std::string &urdf);
            bool loadSRDF(const std::string &srdf);

            bool isGroup(const std::string &name) const;
            std::map<std::string, std::vector<std::string>> &getGroups();
            const std::map<std::string, std::vector<std::string>> &getGroupsConst();
            std::vector<std::string> &getGroupJointNames(const std::string &name);
            const std::vector<std::string> &getGroupJointNamesConst(const std::string &name) const;
            std::vector<dart::dynamics::Joint *> getGroupJoints(const std::string &name) const;
            const std::vector<std::size_t> &getGroupIndices(const std::string &name) const;

            bool addJointToGroup(const std::string &group, const std::string &joint_name);
            bool addLinkToGroup(const std::string &group, const std::string &link_name);
            bool addChainToGroup(const std::string &group, const std::string &tip, const std::string &base);
            bool addGroupToGroup(const std::string &group, const std::string &other);

            void setDof(unsigned int index, double value);

            std::size_t getNumDofsGroup(const std::string &name) const;
            void getGroupState(const std::string &name, Eigen::Ref<Eigen::VectorXd> q) const;
            void setGroupState(const std::string &name, const Eigen::Ref<const Eigen::VectorXd> &q);

        protected:
            bool addNameToGroup(const std::string &group, const std::string &name);
            void processGroup(const std::string &name);

        private:
            std::map<std::string, std::vector<std::string>> groups_;
            std::map<std::string, std::vector<std::size_t>> group_indices_;
        };

        RobotPtr loadMoveItRobot(const std::string &name, const std::string &urdf, const std::string &srdf);

    }  // namespace darts
}  // namespace robowflex

#endif
