/* Author: Carlos Quintero Pena */

#include <robowflex_library/detail/ur5.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_tesseract/conversions.h>
#include <robowflex_tesseract/trajopt_planner.h>

/* \file ur5_custom_planning.cpp
 * Simple demonstration of how to use the TrajOptPlanner with the UR5 with custom terms in the optimization.
 * This customized trajopt planner has end-effector pose constraints at specific timesteps of the trajectory.
 */

using namespace robowflex;
static const std::string GROUP = "manipulator";

class CustomTrajOptPlanner : public TrajOptPlanner
{
public:
    struct CartCnt
    {
        RobotPose pose;
        int timestep;
        std::string link;
        double pos_coeffs;
        double rot_coeffs;
    };

    CustomTrajOptPlanner(const RobotPtr &robot, const std::string &group_name)
      : TrajOptPlanner(robot, group_name, "custom_trajopt")
    {
    }

    PlannerResult plan(const SceneConstPtr &scene, const robot_state::RobotStatePtr &start_state)
    {
        // This is required by any TrajOptPlanner::plan() method.
        ref_state_ = std::make_shared<robot_state::RobotState>(*start_state);

        // Transform robowflex scene to tesseract environment.
        hypercube::sceneToTesseractEnv(scene, env_);

        // Fill in the problem construction info and initialization.
        auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(env_);
        problemConstructionInfo(pci);

        // Add velocity cost.
        addVelocityCost(pci);

        // Add start state.
        addStartState(start_state, pci);

        // Add cartesian terms in constraints_.
        addCartTerms(pci);

        return solve(scene, pci);
    }

    std::vector<CartCnt> constraints_{};

private:
    void addCartTerms(std::shared_ptr<trajopt::ProblemConstructionInfo> pci) const
    {
        for (const auto &term : constraints_)
        {
            Eigen::Quaterniond rotation(term.pose.linear());
            auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
            pose_constraint->term_type = trajopt::TT_CNT;
            pose_constraint->link = term.link;
            pose_constraint->timestep = term.timestep;
            pose_constraint->xyz = term.pose.translation();
            pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
            pose_constraint->pos_coeffs = Eigen::Vector3d::Constant(term.pos_coeffs);
            pose_constraint->rot_coeffs = Eigen::Vector3d::Constant(term.rot_coeffs);
            pose_constraint->name = "pose_cnt_link_" + term.link + std::to_string(term.timestep);

            pci->cnt_infos.push_back(pose_constraint);
        }
    }
};

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Set start state and get the ee pose.
    ur5->setGroupState(GROUP, {0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});
    const auto &ee = ur5->getModel()->getEndEffectors()[0]->getLinkModelNames()[0];
    RobotPose pose = ur5->getLinkTF(ee);

    // Create an RViz visualization helper.
    IO::RVIZHelper rviz(ur5);
    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    // Create and visualize scene.
    auto scene = std::make_shared<Scene>(ur5);
    scene->getCurrentState() = *ur5->getScratchState();
    rviz.updateScene(scene);

    // Create a custom TrajOpt planner
    auto planner = std::make_shared<CustomTrajOptPlanner>(ur5, GROUP);
    planner->initialize(GROUP);
    planner->options.num_waypoints = 17;  // Set number of waypoints in trajectory.

    // Define desired motion of the end-effector.
    EigenSTL::vector_Vector3d directions = {
        {0.0, -0.3, 0.0}, {0.0, 0.0, -0.2}, {0.0, 0.3, 0.0}, {0.0, 0.0, 0.2}};

    // Define a Cartesian constraint for each location.
    for (size_t i = 0; i < directions.size(); ++i)
    {
        CustomTrajOptPlanner::CartCnt term;
        pose.translate(directions[i]);
        term.pose = pose;
        term.timestep = (i + 1) * 4;
        term.link = ee;
        term.pos_coeffs = 1.0;
        term.rot_coeffs = 1.0;

        planner->constraints_.push_back(term);
    }

    // Do planning with cartesian constraints.
    auto response = planner->plan(scene, ur5->getScratchState());
    if (!response.first)
    {
        RBX_INFO("Optimization did not converge");
        return 0;
    }

    // Publish the trajectory to a topic to display in RViz
    Trajectory trajectory(planner->getTrajectory());
    rviz.updateTrajectory(trajectory);

    // Visualize the final pose.
    scene->getCurrentState() = *ur5->getScratchState();
    rviz.updateScene(scene);

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
