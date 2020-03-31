/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_DART_TSR_
#define ROBOWFLEX_DART_TSR_

#include <set>
#include <functional>

#include <ompl/base/Constraint.h>

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/InverseKinematics.hpp>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>

namespace robowflex
{
    namespace darts
    {
        ROBOWFLEX_CLASS_FORWARD(Structure)
        ROBOWFLEX_CLASS_FORWARD(StateSpace)

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

            TSR(const StructurePtr &structure, const std::string &target, const std::string &base,
                const RobotPose &pose, const Bounds &position, const Bounds &orientation);

            TSR(const StructurePtr &structure, const std::string &target, const RobotPose &pose,
                const Bounds &position, const Bounds &orientation);

            // mirrored bounds
            TSR(const StructurePtr &structure, const std::string &target, const std::string &base, //
                const RobotPose &pose, const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

            TSR(const StructurePtr &structure, const std::string &target,  //
                const RobotPose &pose, const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

            // tight bounds
            TSR(const StructurePtr &structure, const std::string &target, const std::string &base,
                const RobotPose &pose);
            TSR(const StructurePtr &structure, const std::string &target, const RobotPose &pose);

            TSR(const StructurePtr &structure, const std::string &target, const std::string &base,
                const Eigen::Vector3d &position, const Eigen::Quaterniond &rotation);
            TSR(const StructurePtr &structure, const std::string &target, const Eigen::Vector3d &position,
                const Eigen::Quaterniond &rotation);

            //
            TSR(const StructurePtr &structure, const std::string &target,
                const std::string &base = magic::ROOT_FRAME);

            void setPose(const RobotPose &pose);
            void setPosition(const Eigen::Ref<const Eigen::Vector3d> &position);
            void setRotation(const Eigen::Quaterniond &orientation);

            std::size_t getDimension() const;
            std::size_t getNumDofs() const;
            void getError(Eigen::Ref<Eigen::VectorXd> vector) const;
            void getGradient(Eigen::VectorXd &q) const;
            void getJacobian(Eigen::Ref<Eigen::MatrixXd> jac) const;

            bool solve();

            void useGroup(const std::string &name);
            void useIndices(const std::vector<std::size_t> &indices);

            StructurePtr getStructure();

        private:
            bool initialize();
            void updateTarget();
            void computeDimension();

            StructurePtr structure_;
            const std::string target_;
            const std::string base_{magic::ROOT_FRAME};

            RobotPose pose_;

            Eigen::Vector3d plower_;
            Eigen::Vector3d pupper_;

            Eigen::Vector3d olower_;
            Eigen::Vector3d oupper_;

            std::shared_ptr<dart::dynamics::SimpleFrame> frame_;
            std::shared_ptr<dart::dynamics::InverseKinematics> ik_;
            dart::dynamics::InverseKinematics::TaskSpaceRegion *tsr_;
            dart::dynamics::InverseKinematics::GradientMethod *grad_;

            std::size_t dimension_;
            std::size_t dofs_;
            std::vector<bool> indices_;
        };

        class TSRConstraint : public ompl::base::Constraint
        {
        public:
            TSRConstraint(const StateSpacePtr &space, const TSRPtr &tsr);

            void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::VectorXd> out) const override;

            void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::MatrixXd> out) const override;

            bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

        protected:
            StateSpacePtr space_;
            TSRPtr tsr_;
        };

        class TSRCompositeConstraint : public ompl::base::Constraint
        {
        public:
            TSRCompositeConstraint(const StateSpacePtr &space, const std::vector<TSRPtr> &tsrs);

            void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::VectorXd> out) const override;

            void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::MatrixXd> out) const override;

            bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

        protected:
            StateSpacePtr space_;
            std::vector<TSRPtr> tsrs_;
            std::set<StructurePtr> structures_;
        };
    }  // namespace darts
}  // namespace robowflex

#endif
