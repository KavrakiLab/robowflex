#include <robowflex_library/robowflex.h>
#include <robowflex_library/detail/ur5.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    UR5Robot ur5;
    ur5.initialize();

    ScenePtr scene(new Scene(ur5));

    std::vector<PlannerPtr> planners;
    OMPL::UR5OMPLPipelinePlannerPtr default_planner(new OMPL::UR5OMPLPipelinePlanner(ur5, "default"));
    default_planner->initialize();
    planners.push_back(default_planner);

    OMPL::Settings settings;
    settings.simplify_solutions = false;
    OMPL::UR5OMPLPipelinePlannerPtr no_simple_planner(new OMPL::UR5OMPLPipelinePlanner(ur5, "simple"));
    no_simple_planner->initialize(settings);
    planners.push_back(no_simple_planner);

    for (auto& planner : planners)
    {
        MotionRequestBuilder request(planner, "manipulator");
        request.setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
        Eigen::Quaterniond orn{0, 0, 1, 0};

        request.setGoalRegion("ee_link", "world",                                         // links
                              pose, Geometry(Geometry::ShapeType::SPHERE, {0.01, 0, 0}),  // position
                              orn, {0.01, 0.01, 0.01}                                     // orientation
                              );

        planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
        if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
            return 1;
    }

    return 0;
}
