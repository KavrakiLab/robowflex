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
        ROBOWFLEX_CLASS_FORWARD(World)
        ROBOWFLEX_CLASS_FORWARD(Structure)
        ROBOWFLEX_CLASS_FORWARD(StateSpace)

        namespace magic
        {
            // static const double DEFAULT_IK_TOLERANCE = 1e-8;
            static const double DEFAULT_IK_TOLERANCE = 1e-4;
            static const std::string ROOT_FRAME = "";
            static const Eigen::Vector3d DEFAULT_IK_TOLERANCES =
                Eigen::Vector3d::Constant(DEFAULT_IK_TOLERANCE);
        }  // namespace magic

        ROBOWFLEX_CLASS_FORWARD(TSR)

        class TSR
        {
            ROBOWFLEX_EIGEN

        public:
            class Specification
            {
            public:
                ROBOWFLEX_EIGEN

                struct
                {
                    std::string structure;
                    std::string frame;
                } target;

                struct
                {
                    std::string structure;
                    std::string frame{magic::ROOT_FRAME};
                } base;

                RobotPose pose{RobotPose::Identity()};

                struct
                {
                    ROBOWFLEX_EIGEN
                    Eigen::Vector3d upper{magic::DEFAULT_IK_TOLERANCES};
                    Eigen::Vector3d lower{-magic::DEFAULT_IK_TOLERANCES};
                } position;

                struct
                {
                    ROBOWFLEX_EIGEN
                    Eigen::Vector3d upper{magic::DEFAULT_IK_TOLERANCES};
                    Eigen::Vector3d lower{-magic::DEFAULT_IK_TOLERANCES};
                } orientation;

                std::size_t dimension{6};
                std::vector<bool> indices{std::vector<bool>(6, true)};

                std::size_t maxIter{50};
                double tolerance{magic::DEFAULT_IK_TOLERANCE};

                Specification() = default;
                Specification(const std::string &structure, const std::string &target,
                              const Eigen::Ref<const Eigen::Vector3d> &position,
                              const Eigen::Quaterniond &rotation);

                void setTarget(const std::string &structure, const std::string &frame);
                void setBase(const std::string &structure, const std::string &frame);
                void setFrame(const std::string &structure, const std::string &target,
                              const std::string &base = magic::ROOT_FRAME);

                void addSuffix(const std::string &suffix);

                void setPosition(const Eigen::Ref<const Eigen::Vector3d> &position);
                void setPosition(double x, double y, double z);
                void setRotation(const Eigen::Quaterniond &orientation);
                void setRotation(double w, double x, double y, double z);
                void setPose(const Eigen::Ref<const Eigen::Vector3d> &position,
                             const Eigen::Quaterniond &rotation);
                void setPose(double xp, double yp, double zp, double wr, double xr, double yr, double zr);

                void setPoseFromWorld(const WorldPtr &world);

                void setXPosTolerance(double lower, double upper);
                void setYPosTolerance(double lower, double upper);
                void setZPosTolerance(double lower, double upper);

                void setXRotTolerance(double lower, double upper);
                void setYRotTolerance(double lower, double upper);
                void setZRotTolerance(double lower, double upper);

                void setNoXPosTolerance();
                void setNoYPosTolerance();
                void setNoZPosTolerance();

                void setNoPosTolerance();

                void setNoXRotTolerance();
                void setNoYRotTolerance();
                void setNoZRotTolerance();

                void setNoRotTolerance();

                Eigen::Vector3d getPosition() const;
                Eigen::Quaterniond getRotation() const;

                bool intersect(const Specification &other);

            private:
                std::size_t getDimension();
                bool isPosConstrained(double lower, double upper);
                bool isRotConstrained(double lower, double upper);
            };

            TSR(const WorldPtr &world, const Specification &spec);
            ~TSR();

            void clear();

            void setWorld(const WorldPtr &world);

            void useGroup(const std::string &name);
            void useIndices(const std::vector<std::size_t> &indices);
            void setWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices);

            std::size_t getSkeletonIndex() const;
            const std::vector<std::size_t> &getIndices() const;
            const std::vector<std::pair<std::size_t, std::size_t>> &getWorldIndices() const;

            std::size_t getDimension() const;
            std::size_t getNumDofs() const;
            std::size_t getNumWorldDofs() const;

            void getErrorWorld(Eigen::Ref<Eigen::VectorXd> error) const;
            void getErrorWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                    Eigen::Ref<Eigen::VectorXd> error) const;
            void getError(const Eigen::Ref<const Eigen::VectorXd> &state,
                          Eigen::Ref<Eigen::VectorXd> error) const;

            void getJacobianWorld(Eigen::Ref<Eigen::MatrixXd> jacobian) const;
            void getJacobianWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                       Eigen::Ref<Eigen::MatrixXd> jacobian) const;
            void getJacobian(const Eigen::Ref<const Eigen::VectorXd> &state,
                             Eigen::Ref<Eigen::MatrixXd> jacobian) const;

            double distanceWorld() const;
            double distanceWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const;
            double distance(const Eigen::Ref<const Eigen::VectorXd> &state) const;

            bool solveWorld();
            bool solveWorldState(Eigen::Ref<Eigen::VectorXd> world);
            bool solve(Eigen::Ref<Eigen::VectorXd> state);

            bool solveGradientWorld();
            bool solveGradientWorldState(Eigen::Ref<Eigen::VectorXd> world);
            bool solveGradient(Eigen::Ref<Eigen::VectorXd> state);

            void setPositionsWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const;
            void setPositions(const Eigen::Ref<const Eigen::VectorXd> &state) const;
            void getPositionsWorldState(Eigen::Ref<Eigen::VectorXd> world) const;
            void getPositions(Eigen::Ref<Eigen::VectorXd> state) const;

            Specification &getSpecification();

            void updatePose();
            void updateBounds();

            void toBijection(Eigen::Ref<Eigen::VectorXd> world,
                             const Eigen::Ref<const Eigen::VectorXd> &state) const;
            void fromBijection(Eigen::Ref<Eigen::VectorXd> state,
                               const Eigen::Ref<const Eigen::VectorXd> &world) const;

            void initialize();

        private:
            void computeBijection();

            WorldPtr world_;
            Specification spec_;

            std::size_t skel_index_;
            std::vector<std::size_t> indices_;
            std::vector<std::pair<std::size_t, std::size_t>> world_indices_;
            std::vector<std::size_t> bijection_;

            std::shared_ptr<dart::dynamics::SimpleFrame> frame_;
            dart::dynamics::BodyNode *tnd_;
            std::shared_ptr<dart::dynamics::InverseKinematics> ik_;
            dart::dynamics::InverseKinematics::TaskSpaceRegion *tsr_;
        };

        ROBOWFLEX_CLASS_FORWARD(TSRSet)
        class TSRSet
        {
        public:
            TSRSet(const WorldPtr &world, const TSRPtr &tsr);
            TSRSet(const WorldPtr &world, const std::vector<TSRPtr> &tsrs);

            void addTSR(const TSRPtr &tsr);
            std::size_t numTSRs() const;

            void setWorld(const WorldPtr &world);
            void addSuffix(const std::string &suffix);

            void useGroup(const std::string &name);
            void useIndices(const std::vector<std::size_t> &indices);
            void setWorldIndices(const std::vector<std::pair<std::size_t, std::size_t>> &indices);

            std::size_t getDimension() const;

            void getErrorWorld(Eigen::Ref<Eigen::VectorXd> error) const;
            void getErrorWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                    Eigen::Ref<Eigen::VectorXd> error) const;

            void getJacobianWorldState(const Eigen::Ref<const Eigen::VectorXd> &world,
                                       Eigen::Ref<Eigen::MatrixXd> jacobian) const;

            double distanceWorld() const;
            double distanceWorldState(const Eigen::Ref<const Eigen::VectorXd> &world) const;

            bool solveWorld();
            bool solveWorldState(Eigen::Ref<Eigen::VectorXd> world);

            bool solveGradientWorldState(Eigen::Ref<Eigen::VectorXd> world);

            double getTolerance() const;

            void initialize();

        private:
            WorldPtr world_;
            std::set<std::size_t> skel_indices_;

            double tolerance_{magic::DEFAULT_IK_TOLERANCE};
            std::size_t maxIter_{50};

            std::vector<TSRPtr> tsrs_;
            std::size_t dimension_{0};
        };

        ROBOWFLEX_CLASS_FORWARD(TSRConstraint)
        class TSRConstraint : public ompl::base::Constraint
        {
        public:
            TSRConstraint(const StateSpacePtr &space, const TSRPtr &tsr);
            TSRConstraint(const StateSpacePtr &space, const std::vector<TSRPtr> &tsrs);
            TSRConstraint(const StateSpacePtr &space, const TSRSetPtr &tsr);

            void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::VectorXd> out) const override;

            void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::MatrixXd> out) const override;

            bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

            TSRSetPtr getSet();

        protected:
            StateSpacePtr space_;
            TSRSetPtr tsr_;
        };
    }  // namespace darts
}  // namespace robowflex

#endif
