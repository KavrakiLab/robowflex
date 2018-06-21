#include <robowflex_library/robowflex.h>
#include <robowflex_library/detail/ur5.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    auto ur5 = std::make_shared<UR5Robot>();
    ur5->initialize();

    auto scene = std::make_shared<Scene>(ur5);

    auto planner = std::make_shared<OMPL::UR5OMPLPipelinePlanner>(ur5);
    planner->initialize();

    MotionRequestBuilderPtr joint_request(new MotionRequestBuilder(planner, "manipulator"));
    joint_request->setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});
    joint_request->setGoalConfiguration({-0.39, -0.69, -2.12, 2.82, -0.39, 0});

    MotionRequestBuilderPtr pose_request(new MotionRequestBuilder(planner, "manipulator"));
    pose_request->setStartConfiguration({0.0677, -0.8235, 0.9860, -0.1624, 0.0678, 0.0});

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(Eigen::Vector3d{-0.268, -0.826, 1.313});
    Eigen::Quaterniond orn{0, 0, 1, 0};

    pose_request->setGoalRegion("ee_link", "world",                                         // links
                               pose, Geometry(Geometry::ShapeType::SPHERE, {0.01, 0, 0}),  // position
                               orn, {0.01, 0.01, 0.01}                                     // orientation
    );

    Benchmarker benchmark;
    benchmark.addBenchmarkingRequest("joint", scene, planner, joint_request);
    benchmark.addBenchmarkingRequest("pose", scene, planner, pose_request);

    benchmark.benchmark({std::make_shared<OMPLBenchmarkOutputter>("robowflex_ur5_test/")});

    return 0;
}
