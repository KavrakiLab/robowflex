/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/detail/ur5.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create the default UR5 robot.
    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    // Dump the geometry information for visualization.
    ur5->dumpGeometry("ur5.yml");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(ur5);

    // Create the default planner for the UR5.
    auto default_planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5, "default");
    default_planner->initialize();

    // Create the a planner for the UR5, and disable simplification.
    auto simple_planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5, "simple");

    OMPL::Settings settings;
    settings.simplify_solutions = false;

    simple_planner->initialize(settings);

    // Run a motion plan for each planner.
    for (auto &planner : {default_planner, simple_planner})
    {
        // Create a motion planning request with a pose goal.
        MotionRequestBuilder request(planner, "manipulator");
        request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
        Eigen::Quaterniond orn{0, 0, 1, 0};

        request.setGoalRegion("ee_link", "world",               // links
                              pose, Geometry::makeSphere(0.1),  // position
                              orn, {0.01, 0.01, 0.01}           // orientation
                              );

        // Do motion planning!
        planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;

        // Output transforms from path to a file for visualization.
        ur5->dumpPathTransforms(*res.trajectory_, "ur5_path_" + planner->getName() + ".yml");
    }

    return 0;
}
