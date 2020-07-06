/* Author: Zachary Kingston */

#include <thread>
#include <chrono>

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/gui.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    auto world = std::make_shared<darts::World>();

    auto fetch1 = darts::loadMoveItRobot("fetch1",                                         //
                                         "package://fetch_description/robots/fetch.urdf",  //
                                         "package://fetch_moveit_config/config/fetch.srdf");
    auto start = fetch1->getSkeleton()->getState();
    world->addRobot(fetch1);

    darts::Window window(world);

    darts::TSR::Specification spec;
    spec.setFrame("fetch1", "wrist_roll_link");
    window.addWidget(std::make_shared<darts::TSRWidget>("EE", spec));

    window.run();
    return 0;
}
