/* Author: Carlos Quintero Pena*/

#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_tesseract/stretch.h>
// #include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/yaml.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;

static const std::string GROUP = "mobile_base_manipulator";
// static const std::string GROUP = "stretch_arm";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    bool enable_mobile_base = false;
    std::cout << "CQ1" << std::endl;
    if (GROUP == "mobile_base_manipulator")
        enable_mobile_base = true;

    // Create the default Fetch robot.
    auto stretch = std::make_shared<StretchRobot>();
    stretch->initialize(false, enable_mobile_base);
    auto const &model = stretch->getModelConst();
    auto const &gmodel = model->getJointModelGroup(GROUP);
    auto names = gmodel->getJointModelNames();

    for (auto const &name : names)
        std::cout << name << std::endl;
    stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 1.0});
    // stretch->setGroupState(GROUP, {0.2, 0.0, 0.0, 0.0, 0.0, 0.0});

    // std::cout << stretch->getURDFString() << std::endl;
    // std::cout << stretch->getSRDFString() << std::endl;
    std::cout << "CQ2" << std::endl;

    // Load tabletop scene.
    auto scene = std::make_shared<Scene>(stretch);
    // scene->fromYAMLFile("package://robowflex_tesseract/scenes/table/scene.yaml");
    scene->getCurrentState() = *stretch->getScratchState();

    // auto robot_offset = TF::createPoseQ(Eigen::Vector3d{0.5, 0.0, 0}, Eigen::Quaterniond{0.707, 0, 0, -0.707});//-90 z
    // auto robot_offset = TF::createPoseQ(Eigen::Vector3d{-0.5, 0.0, 0}, Eigen::Quaterniond{1,0,0,0});
    // scene->moveAllObjectsGlobal(robot_offset);

    // RVIZ helper.
    const auto &rviz = std::make_shared<IO::RVIZHelper>(stretch);
    rviz->updateScene(scene);
    // rviz->visualizeState(request->getStartConfiguration());

    // RBX_INFO("Visualizing start state");
    RBX_INFO("Press Enter to continue");
    std::cin.ignore();


    // Create the default planner for the Fetch.
    auto planner = std::make_shared<OMPL::StretchOMPLPipelinePlanner>(stretch, "default");
    planner->initialize();


/*
    // Attach object to end effector.
    // scene->attachObject(*fetch->getScratchState(), "Can1");

    // Create a TrajOpt planner for Setch.
    // std::cout << "CQ3" << std::endl;
    auto planner = std::make_shared<TrajOptPlanner>(stretch, GROUP);
    // planner->initialize("torso_lift_link", "gripper_link");
    // std::cout << "CQ4" << std::endl;
    // planner->initialize(GROUP);
    // planner->initialize("base_link", "link_wrist_yaw", true);
    planner->initialize("link_mast", "link_wrist_yaw", enable_mobile_base);
    // planner->initialize("base_link", "link_head_tilt");
// std::cout << "CQ5" << std::endl;
    // Set planner parameters.
    planner->options.num_waypoints = 30;  // Select number of waypoints in trajectory
    planner->setInitType(trajopt::InitInfo::Type::JOINT_INTERPOLATED);  // Initialize using a straight-line
                                                                        // between start and goal in C-space.
    // planner->options.joint_vel_coeffs = 60.0;
    planner->options.return_first_sol = false;
    // planner->fixJoints({"base_joint/theta", "joint_wrist_yaw", "base_joint/y"});
    // planner->fixJoints({"joint_wrist_yaw"});

    // planner->options.trust_box_size = 0.5;
    // planner->options.joint_state_safety_margin_coeffs = 100.0;
    // planner->options.return_after_timeout = true;

    std::cout << "links:" << std::endl;
    auto const &manip_links = planner->getManipulatorLinks();
    for (const auto &link : manip_links)
        std::cout << link << std::endl;
    std::cout << "" << std::endl;
    std::cout << "joints:" << std::endl;
    auto const &manip_joints = planner->getManipulatorJoints();
    for (const auto &joint : manip_joints)
        std::cout << joint << std::endl;
*/
    // Load request.
    const auto &request = std::make_shared<MotionRequestBuilder>(stretch, GROUP);
    request->setStartConfiguration(stretch->getScratchState());

    stretch->setGroupState(GROUP, {0.5, 0.5, 1.57, 0.9, 0.1, 0.1, 0.1, 0.1, 1.0});
    // stretch->setGroupState(GROUP, {0.9, 0.1, 0.1, 0.1, 0.1, 1.0});
    request->setGoalConfiguration(stretch->getScratchState());
    scene->getCurrentState() = *stretch->getScratchState();
    rviz->updateScene(scene);

    // Do motion planning.
    const auto &res = planner->plan(scene, request->getRequest());
    // if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    // {
        // RBX_ERROR("Trajectory not found. Aborting!");
        // return 1;
    // }

    Trajectory traj(res.trajectory_);
    traj.computeTimeParameterization();
    // traj.interpolate(80);
    rviz->updateTrajectory(traj);

    RBX_INFO("Press Enter to exit");
    std::cin.ignore();

    return 0;

    /*
    // Startup ROS
    ROS ros(argc, argv);


    std::cout << "CQ1" << std::endl;
    // Create the default Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();
    auto const &model = fetch->getModelConst();
    auto const &gmodel = model->getJointModelGroup(GROUP);
    auto names = gmodel->getJointModelNames();

    for (auto const &name : names)
        std::cout << name << std::endl;
    fetch->setGroupState(GROUP, {0.201, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});

    // std::cout << stretch->getURDFString() << std::endl;
    // std::cout << stretch->getSRDFString() << std::endl;
    std::cout << "CQ2" << std::endl;

    // Load tabletop scene.
    auto scene = std::make_shared<Scene>(fetch);
    scene->getCurrentState() = *fetch->getScratchState();
    // scene->fromYAMLFile("package://robowflex_tesseract/scenes/table/scene.yaml");

    // Attach object to end effector.
    // scene->attachObject(*fetch->getScratchState(), "Can1");

    // Create a TrajOpt planner for Setch.
    std::cout << "CQ3" << std::endl;
    auto planner = std::make_shared<TrajOptPlanner>(fetch, GROUP);
    // planner->initialize("torso_lift_link", "gripper_link");
    std::cout << "CQ4" << std::endl;
    planner->initialize(GROUP);
std::cout << "CQ5" << std::endl;
    // Set planner parameters.
    planner->options.num_waypoints = 8;  // Select number of waypoints in trajectory
    planner->setInitType(trajopt::InitInfo::Type::JOINT_INTERPOLATED);  // Initialize using a straight-line
                                                                        // between start and goal in C-space.
    std::cout << "links:" << std::endl;
    auto const &manip_links = planner->getManipulatorLinks();
    for (const auto &link : manip_links)
        std::cout << link << std::endl;
    std::cout << "joints:" << std::endl;
    auto const &manip_joints = planner->getManipulatorJoints();
    for (const auto &joint : manip_joints)
        std::cout << joint << std::endl;

    // Load request.
    const auto &request = std::make_shared<MotionRequestBuilder>(fetch, GROUP);
    request->setStartConfiguration(fetch->getScratchState());

    // request->fromYAMLFile("package://robowflex_tesseract/scenes/table/request.yaml");

    // RVIZ helper.
    const auto &rviz = std::make_shared<IO::RVIZHelper>(fetch);
    rviz->updateScene(scene);
    // rviz->visualizeState(request->getStartConfiguration());

    // RBX_INFO("Visualizing start state");
    RBX_INFO("Press Enter to continue");
    std::cin.ignore();
    fetch->setGroupState(GROUP, {1.301, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007});
    request->setGoalConfiguration(fetch->getScratchState());

    // Do motion planning.
    const auto &res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        rviz->updateTrajectory(res);

    // rviz->visualizeState(request->getGoalConfiguration());

    // RBX_INFO("Visualizing goal state");
    RBX_INFO("Press Enter to exit");
    std::cin.ignore();
*/
    return 0;

}
