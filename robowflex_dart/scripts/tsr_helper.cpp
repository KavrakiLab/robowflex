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

    return 0;
}
