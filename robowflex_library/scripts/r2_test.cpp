#include <robowflex_library/util.h>
#include <robowflex_library/geometry.h>
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
    r2->initialize({"legsandtorso"});

    // Dump the geometry information for visualization.
    r2->dumpGeometry("r2.yml");

    // Create an empty scene.
    auto scene = std::make_shared<Scene>(r2);

    // Create the default motion planner for R2.
    auto planner = std::make_shared<OMPL::R2OMPLPipelinePlanner>(r2);
    planner->initialize();

    // Load a motion planning request (a step with a torso constraint).
    MotionRequestBuilder request(planner, "legsandtorso");
    request.fromYAMLFile("package://robowflex_library/yaml/r2_plan_waist.yml");

    // Clear path constraints so we can rebuild them.
    request.getPathConstraints().position_constraints.clear();
    request.getPathConstraints().orientation_constraints.clear();

    // Set the scratch state of the robot.
    r2->setState(request.getRequest().start_state);

    const std::string world = "world";
    const std::string waist = "r2/waist_center";
    const std::string left_foot = "r2/left_leg/gripper/tip";
    Eigen::Vector3d tolerances(0.01, 0.01, 0.01);

    // Set a pose constraint on the left foot (keep fixed throughout the path).
    auto foot_tf = r2->getLinkTF(left_foot);
    request.addPathPoseConstraint(           //
        left_foot, world,                    //
        foot_tf, Geometry::makeSphere(0.1),  //
        Eigen::Quaterniond(foot_tf.rotation()), tolerances);

    // Set a orientation constraint on the waist (to keep it up throughout the path)
    auto waist_tf = r2->getRelativeLinkTF(left_foot, waist);
    request.addPathOrientationConstraint(  //
        waist, left_foot,                  //
        Eigen::Quaterniond(waist_tf.rotation()), tolerances);

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Output transforms from path to a file for visualization.
    r2->dumpPathTransforms(*res.trajectory_, "r2_path.yml", 30, 0.5);

    return 0;
}
