/* Author: Zachary Kingston */

#include <robowflex_dart/io.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/world.h>

using namespace robowflex;

int main()
{
    darts::IO::addPackage("r2_description",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_description/");
    darts::IO::addPackage("r2_moveit_config",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_moveit_config/");

    darts::IO::addPackage("fetch_description",  //
                          "/home/zak/ros/melodic/system/src/fetch_description/");
    darts::IO::addPackage("fetch_moveit_config",  //
                          "/home/zak/ros/melodic/system/src/fetch_moveit_config/");

    auto fetch = darts::loadMoveItRobot("fetch",                                          //
                                        "package://fetch_description/robots/fetch.urdf",  //
                                        "package://fetch_moveit_config/config/fetch.srdf");
    fetch->setDof(4, 0.3);

    auto fetch2 = fetch->clone("fetch2");
    fetch2->setDof(4, 1.0);

    auto r2 = darts::loadMoveItRobot("r2",                                       //
                                     "package://r2_description/urdf/r2c6.urdf",  //
                                     "package://r2_moveit_config/config/r2.srdf");

    r2->setDof(4, -0.3);
    r2->setDof(5, 1.5);

    darts::World world;

    auto environment = std::make_shared<darts::Structure>("ground");
    environment->addGround();

    world.addRobot(fetch);
    world.addRobot(fetch2);
    world.addRobot(r2);
    world.addStructure(environment);

    world.openOSGViewer();
    return 0;
}
