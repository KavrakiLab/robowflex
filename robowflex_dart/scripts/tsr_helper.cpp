/* Author: Zachary Kingston */

#include <chrono>
#include <thread>

#include <robowflex_library/io/yaml.h>

#include <robowflex_dart/gui.h>
#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    int n = 1;
    if (argc > 1)
        n = std::atoi(argv[1]);

    if (n < 3)
    {
        auto world = std::make_shared<darts::World>();

        auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                             "package://fetch_description/robots/fetch.urdf",  //
                                             "package://fetch_moveit_config/config/fetch.srdf");
        auto start = fetch1->getSkeleton()->getState();
        world->addRobot(fetch1);

        darts::Window window(world);

        if (n == 2)
        {
            darts::TSR::Specification spec1;
            spec1.setFrame("fetch1", "wrist_roll_link", "base_link");
            spec1.pose = fetch1->getFrame("wrist_roll_link")->getWorldTransform();
            spec1.setXPosTolerance(0.05);
            spec1.setYPosTolerance(0.05);
            spec1.setZPosTolerance(0.05);
            // spec1.setNoRotTolerance();
            auto ew1 = std::make_shared<darts::TSREditWidget>("EE", spec1);
            window.addWidget(ew1);

            darts::TSR::Specification spec2;
            spec2.setFrame("fetch1", "elbow_flex_link", "base_link");
            spec2.pose = fetch1->getFrame("elbow_flex_link")->getWorldTransform();
            // spec2.setNoRotTolerance();
            spec2.setXPosTolerance(0.05);
            spec2.setYPosTolerance(0.05);
            spec2.setZPosTolerance(0.05);
            auto ew2 = std::make_shared<darts::TSREditWidget>("Elbow", spec2);
            window.addWidget(ew2);

            std::vector<darts::TSRPtr> tsrs{ew1->getTSR(), ew2->getTSR()};
            auto sw = std::make_shared<darts::TSRSolveWidget>(world, tsrs);
            window.addWidget(sw);
        }
        else
        {
            darts::TSR::Specification spec;
            spec.setFrame("fetch1", "wrist_roll_link", "base_link");
            spec.pose = fetch1->getFrame("wrist_roll_link")->getWorldTransform();
            auto ew = std::make_shared<darts::TSREditWidget>("EE", spec);
            window.addWidget(ew);

            std::vector<darts::TSRPtr> tsrs{ew->getTSR()};
            auto sw = std::make_shared<darts::TSRSolveWidget>(world, tsrs);
            window.addWidget(sw);
        }

        window.run();
    }
    else
    {
        darts::IO::addPackage("r2_description",  //
                              "/home/zak/old_ros/melodic/r2/src/r2_description/");
        darts::IO::addPackage("r2_moveit_config",  //
                              "/home/zak/old_ros/melodic/r2/src/r2_moveit_config/");

        auto world = std::make_shared<darts::World>();

        auto r2 = darts::loadMoveItRobot("r2",                                       //
                                         "package://r2_description/urdf/r2c6.urdf",  //
                                         "package://r2_moveit_config/config/r2.srdf");

        world->addRobot(r2);

        moveit_msgs::MotionPlanRequest message;
        IO::fromYAMLFile(message, "package://robowflex_library/yaml/r2_plan_waist.yml");

        darts::PlanBuilder builder(world);
        builder.getStartFromMessage("r2", message);

        darts::Window window(world);

        darts::TSR::Specification waist_spec;
        waist_spec.setFrame("r2", "r2/waist_center");
        waist_spec.setPoseFromWorld(world);
        auto ew1 = std::make_shared<darts::TSREditWidget>("Waist", waist_spec);
        window.addWidget(ew1);

        darts::TSR::Specification lleg_spec;
        lleg_spec.setTarget("r2", "r2/left_leg/gripper/tip");
        lleg_spec.setPoseFromWorld(world);
        auto ew2 = std::make_shared<darts::TSREditWidget>("LLeg", lleg_spec);
        window.addWidget(ew2);

        darts::TSR::Specification rleg_spec;
        rleg_spec.setTarget("r2", "r2/right_leg/gripper/tip");
        rleg_spec.setPoseFromWorld(world);
        auto ew3 = std::make_shared<darts::TSREditWidget>("RLeg", rleg_spec);
        window.addWidget(ew3);

        auto tsrs = std::make_shared<darts::TSRSet>(world, ew1->getTSR());
        tsrs->addTSR(ew2->getTSR(), false, 0.5);
        tsrs->addTSR(ew3->getTSR(), false, 0.5);
        tsrs->useGroup("legsandtorso");
        tsrs->setStep(0.1);

        auto sw = std::make_shared<darts::TSRSolveWidget>(tsrs);
        window.addWidget(sw);

        window.run();
    }

    return 0;
}
