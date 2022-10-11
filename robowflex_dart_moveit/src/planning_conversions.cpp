/* Author: Zachary Kingston */

#include <robowflex_moveit/utility/conversions.h>

#include <robowflex_dart_moveit/planning_conversions.h>

using namespace robowflex::darts;

void conversions::getWorkspaceBoundsFromMessage(PlanBuilder &builder,
                                                const moveit_msgs::MotionPlanRequest &msg)
{
    auto &world = builder.world;

    const auto &mic = msg.workspace_parameters.min_corner;
    world->getWorkspaceLow()[0] = mic.x;
    world->getWorkspaceLow()[1] = mic.y;
    world->getWorkspaceLow()[2] = mic.z;

    const auto &mac = msg.workspace_parameters.max_corner;
    world->getWorkspaceHigh()[0] = mac.x;
    world->getWorkspaceHigh()[1] = mac.y;
    world->getWorkspaceHigh()[2] = mac.z;
}

void conversions::getGroupFromMessage(PlanBuilder &builder, const std::string &robot_name,
                                      const moveit_msgs::MotionPlanRequest &msg)
{
    bool cyclic = false;
    if (not msg.path_constraints.position_constraints.empty() or
        not msg.path_constraints.orientation_constraints.empty())
        cyclic = true;

    builder.addGroup(robot_name, msg.group_name, cyclic ? 2 : 0);
}

void conversions::getStartFromMessage(PlanBuilder &builder, const std::string &robot_name,
                                      const moveit_msgs::MotionPlanRequest &msg)
{
    auto &world = builder.world;
    auto robot = world->getRobot(robot_name);

    for (std::size_t i = 0; i < msg.start_state.joint_state.name.size(); ++i)
    {
        std::string name = msg.start_state.joint_state.name[i];
        double value = msg.start_state.joint_state.position[i];

        robot->setJoint(name, value);
    }

    builder.setStartConfigurationFromWorld();
}

JointRegionGoalPtr conversions::fromJointConstraints(PlanBuilder &builder,
                                                     const std::vector<moveit_msgs::JointConstraint> &msgs)
{
    auto &rspace = builder.rspace;

    std::size_t n = rspace->getDimension();
    Eigen::VectorXd lower(n);
    Eigen::VectorXd upper(n);

    for (const auto &msg : msgs)
    {
        const auto &joint = rspace->getJoint(msg.joint_name);
        joint->getSpaceVars(lower)[0] = msg.position - msg.tolerance_below;
        joint->getSpaceVars(upper)[0] = msg.position + msg.tolerance_above;
    }

    return std::make_shared<JointRegionGoal>(builder, lower, upper);
}

TSRPtr conversions::fromPositionConstraint(WorldPtr &world, const std::string &robot_name,
                                           const moveit_msgs::PositionConstraint &msg)
{
    TSR::Specification spec;
    spec.setFrame(robot_name, msg.link_name);
    if (not msg.header.frame_id.empty() and msg.header.frame_id != "world")
        spec.setBase(robot_name, msg.header.frame_id);

    if (not msg.constraint_region.meshes.empty())
    {
        // XROS_ERROR("Invalid Position Constraint");
        return nullptr;
    }

    if (msg.constraint_region.primitives.size() != 1)
    {
        // XROS_ERROR("Invalid Position Constraint");
        return nullptr;
    }

    spec.setNoRotTolerance();

    // get pose
    spec.setPose(TF::poseMsgToEigen(msg.constraint_region.primitive_poses[0]));
    spec.setPosition(spec.getPosition() + TF::vectorMsgToEigen(msg.target_point_offset));

    auto &&primitive = msg.constraint_region.primitives[0];
    if (primitive.type == shape_msgs::SolidPrimitive::BOX)
    {
        spec.setXPosTolerance(primitive.dimensions[0]);
        spec.setYPosTolerance(primitive.dimensions[1]);
        spec.setZPosTolerance(primitive.dimensions[2]);
    }
    else if (primitive.type == shape_msgs::SolidPrimitive::SPHERE)
    {
        spec.setXPosTolerance(primitive.dimensions[0]);
        spec.setYPosTolerance(primitive.dimensions[0]);
        spec.setZPosTolerance(primitive.dimensions[0]);
    }

    return std::make_shared<TSR>(world, spec);
}

TSRPtr conversions::fromOrientationConstraint(WorldPtr &world, const std::string &robot_name,
                                              const moveit_msgs::OrientationConstraint &msg)
{
    TSR::Specification spec;
    spec.setFrame(robot_name, msg.link_name);
    if (not msg.header.frame_id.empty() and msg.header.frame_id != "world")
        spec.setBase(robot_name, msg.header.frame_id);

    spec.setRotation(TF::quaternionMsgToEigen(msg.orientation));
    spec.setNoPosTolerance();
    spec.setXRotTolerance(msg.absolute_x_axis_tolerance);
    spec.setYRotTolerance(msg.absolute_y_axis_tolerance);
    spec.setZRotTolerance(msg.absolute_z_axis_tolerance);

    return std::make_shared<TSR>(world, spec);
}

void conversions::getPathConstraintsFromMessage(PlanBuilder &builder, const std::string &robot_name,
                                                const moveit_msgs::MotionPlanRequest &msg)
{
    for (const auto &constraint : msg.path_constraints.position_constraints)
        builder.addConstraint(fromPositionConstraint(builder.world, robot_name, constraint));
    for (const auto &constraint : msg.path_constraints.orientation_constraints)
        builder.addConstraint(fromOrientationConstraint(builder.world, robot_name, constraint));
}

ompl::base::GoalPtr conversions::getGoalFromMessage(PlanBuilder &builder, const std::string &robot_name,
                                                    const moveit_msgs::MotionPlanRequest &msg)
{
    // TODO get other goals as well
    std::vector<TSRPtr> tsrs;
    for (const auto &constraint : msg.goal_constraints[0].position_constraints)
        tsrs.emplace_back(fromPositionConstraint(builder.world, robot_name, constraint));
    for (const auto &constraint : msg.goal_constraints[0].orientation_constraints)
        tsrs.emplace_back(fromOrientationConstraint(builder.world, robot_name, constraint));

    if (tsrs.empty())
        return fromJointConstraints(builder, msg.goal_constraints[0].joint_constraints);

    return builder.getGoalTSR(tsrs);
}

ompl::base::GoalPtr conversions::fromMessage(PlanBuilder &builder, const std::string &robot_name,
                                             const moveit_msgs::MotionPlanRequest &msg)
{
    getWorkspaceBoundsFromMessage(builder, msg);
    getGroupFromMessage(builder, robot_name, msg);
    getStartFromMessage(builder, robot_name, msg);
    getPathConstraintsFromMessage(builder, robot_name, msg);
    builder.initialize();

    auto goal = getGoalFromMessage(builder, robot_name, msg);
    builder.setGoal(goal);

    return goal;
}
