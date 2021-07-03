/* Author: Constantinos Chamzas */

#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <robowflex_ompl/ompl_trajectory.h>

using namespace robowflex;

OMPL::OMPLTrajectory::OMPLTrajectory(const RobotConstPtr &robot, const std::string &group)
  : Trajectory(robot, group)
{
}

OMPL::OMPLTrajectory::OMPLTrajectory(robot_trajectory::RobotTrajectory &trajectory) : Trajectory(trajectory)
{
}

ompl::geometric::PathGeometric OMPL::OMPLTrajectory::toOMPLPath(const ompl::geometric::SimpleSetupPtr &ss)
{
    auto path = ompl::geometric::PathGeometric(ss->getSpaceInformation());
    auto *tstate = ss->getSpaceInformation()->allocState();
    auto traj_msg = getMessage();

    // transform to Path geometric
    for (const auto &state_vec : traj_msg.joint_trajectory.points)
        for (unsigned int j = 0; j < state_vec.positions.size(); j++)
            tstate->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[j] = state_vec.positions[j];

    path.append(tstate);

    return path;
}

void OMPL::OMPLTrajectory::fromOMPLPath(const robot_state::RobotState &reference_state,
                                        const ompl::geometric::PathGeometric &path)
{
    const auto &mbss = std::dynamic_pointer_cast<ompl_interface::ModelBasedStateSpace>(
        path.getSpaceInformation()->getStateSpace());

    if (not mbss)
        throw Exception(1, "Failed to extract StateSpace from provided OMPL path!");

    moveit::core::RobotState ks = reference_state;
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        mbss->copyToRobotState(ks, path.getState(i));
        trajectory_->addSuffixWayPoint(ks, 0.0);
    }
}
