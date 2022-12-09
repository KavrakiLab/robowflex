/* Author: Zachary Kingston */

#include <chrono>
#include <thread>

#include <robowflex_library/log.h>
#include <robowflex_library/random.h>

#include <robowflex_dart/gui.h>
#include <robowflex_dart/io.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/acm.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    auto world = std::make_shared<darts::World>();

    auto fetch = darts::loadMoveItRobot("fetch",                                                  //
                                        "package://robowflex_resources/fetch/robots/fetch.urdf",  //
                                        "package://robowflex_resources/fetch/config/fetch.srdf");

    // Reparent Fetch with a planar joint
    auto *base_link = fetch->getFrame("base_link");
    dart::dynamics::PlanarJoint::Properties planar_joint;
    planar_joint.mName = "planar_joint";

    base_link->moveTo<dart::dynamics::PlanarJoint>(nullptr, planar_joint);

    // Construct new group out of base and head
    fetch->addJointToGroup("observation", "planar_joint");
    fetch->addJointToGroup("observation", "head_pan_joint");
    fetch->addJointToGroup("observation", "head_tilt_joint");

    world->addRobot(fetch);

    // Add obstacle
    auto scene = std::make_shared<darts::Structure>("object");
    scene->addGround(-0.01, 10);

    size_t nobs = 20;

    for (size_t i = 0; i < nobs; ++i)
    {
        dart::dynamics::WeldJoint::Properties box_joint;
        box_joint.mName = log::format("box%1%", i);
        box_joint.mT_ParentBodyToJoint.translation() =
            Eigen::Vector3d(RNG::uniformReal(-4, 4), RNG::uniformReal(-4, 4), 0.5);
        scene->addWeldedFrame(box_joint, darts::makeBox(0.5, 0.5, 1));
    }

    for (size_t i = 0; i < nobs; ++i)
        for (size_t j = i + 1; j < nobs; ++j)
            scene->getACM()->disableCollision(log::format("box%1%", i), log::format("box%1%", j));

    world->addStructure(scene);

    // Planning

    darts::PlanBuilder builder(world);
    builder.addGroup("fetch", "observation");

    builder.setStartConfiguration({
        -5, -5, 1.57, 0., 0.,  //
    });

    builder.initialize();

    auto goal = builder.getGoalConfiguration({
        5, 5, -1.57, 0., 0.,  //
    });

    builder.setGoal(goal);

    auto rrt = std::make_shared<ompl::geometric::RRTstar>(builder.info);
    builder.ss->setPlanner(rrt);

    builder.setup();
    builder.ss->print();

    darts::Window window(world);
    window.run([&] {
        ompl::base::PlannerStatus solved = builder.ss->solve(5.0);

        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath(), 10);
        }
        else
            RBX_WARN("No solution found");
    });

    return 0;
}
