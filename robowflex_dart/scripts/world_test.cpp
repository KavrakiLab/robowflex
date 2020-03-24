/* Author: Zachary Kingston */

#include <se3ez/io.h>
#include <se3ez/robot.h>

int main()
{
    se3ez::io::addPackage("r2_description",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_description/");
    se3ez::io::addPackage("r2_moveit_config",  //
                          "/home/zak/old_ros/melodic/r2/src/r2_moveit_config/");

    se3ez::io::addPackage("fetch_description",  //
                          "/home/zak/ros/melodic/system/src/fetch_description/");
    se3ez::io::addPackage("fetch_moveit_config",  //
                          "/home/zak/ros/melodic/system/src/fetch_moveit_config/");

    auto fetch = se3ez::loadMoveItRobot("fetch",                                          //
                                        "package://fetch_description/robots/fetch.urdf",  //
                                        "package://fetch_moveit_config/config/fetch.srdf");
    fetch->setDof(4, 0.3);

    auto fetch2 = fetch->clone("fetch2");
    fetch2->setDof(4, 1.0);

    auto r2 = se3ez::loadMoveItRobot("r2",                                       //
                                     "package://r2_description/urdf/r2c6.urdf",  //
                                     "package://r2_moveit_config/config/r2.srdf");

    r2->setDof(4, -0.3);
    r2->setDof(5, 1.5);

    se3ez::World world;

    auto environment = std::make_shared<se3ez::Environment>("ground");
    environment->addGround();

    world.addRobot(fetch);
    world.addRobot(fetch2);
    world.addRobot(r2);
    world.addEnvironment(environment);

    world.openOSGViewer();
    return 0;
}
