#include <robowflex_library/util.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/detail/r2.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS.
    startROS(argc, argv);

    // Create an R2 robot, initialize the `legsandtorso` kinematics solver.
    auto r2 = std::make_shared<R2Robot>();
    r2->initialize({});

    // Dump the geometry information for visualization.
    r2->dumpGeometry("r2.yml");

    auto trajectory = r2->loadSMTData("~/Downloads/planning_testing/2018_07_09_091322_left_foot_grab.hdf5");

    // Output transforms from path to a file for visualization.
    r2->dumpPathTransforms(*trajectory, "r2_path.yml", 30, 0.5);

    return 0;
}
