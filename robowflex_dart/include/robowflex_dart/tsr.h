/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_TSR_
#define ROBOWFLEX_DART_TSR_

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>

namespace robowflex
{
    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(Structure)

        namespace magic
        {
            static const double DEFAULT_IK_TOLERANCE = 1e-4;
            static const std::string ROOT_FRAME = "";
        }  // namespace magic

        ROBOWFLEX_CLASS_FORWARD(TSR)

        class TSR
        {
            ROBOWFLEX_EIGEN

        public:
            using Bounds = std::pair<Eigen::Vector3d, Eigen::Vector3d>;

            TSR(const std::string &target, const std::string &base, const RobotPose &pose,  //
                const Bounds &position, const Bounds &orientation);

            TSR(const std::string &target, const RobotPose &pose,  //
                const Bounds &position, const Bounds &orientation);

            // tight bounds
            TSR(const std::string &target, const std::string &base, const RobotPose &pose);
            TSR(const std::string &target, const RobotPose &pose);

            //
            TSR(const std::string &target, const std::string &base = magic::ROOT_FRAME);

            void setPose(const RobotPose &pose);
            void setPosition(const Eigen::Ref<const Eigen::Vector3d> &position);
            void setRotation(const Eigen::Quaterniond &orientation);

            bool setIKTarget(StructurePtr structure);
            bool solve(StructurePtr structure);

        private:
            std::string target_;
            std::string base_{magic::ROOT_FRAME};

            RobotPose pose_;

            Eigen::Vector3d plower_;
            Eigen::Vector3d pupper_;

            Eigen::Vector3d olower_;
            Eigen::Vector3d oupper_;
        };
    }  // namespace darts
}  // namespace robowflex

#endif
