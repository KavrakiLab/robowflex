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
#include <robowflex_tesseract/conversions.h>

using namespace robowflex;

static const std::string GROUP = "mobile_base_manipulator";
// static const std::string GROUP = "stretch_head";
// static const std::string GROUP = "stretch_arm";

// class CustomTrajOptPlanner : public TrajOptPlanner
// {
// public:
//     struct CartCnt
//     {
//         RobotPose pose;
//         int timestep;
//         std::string link;
//         double pos_coeffs;
//         double rot_coeffs;
//     };
//
//     CustomTrajOptPlanner(const RobotPtr &robot, const std::string &group_name)
//       : TrajOptPlanner(robot, group_name, "custom_trajopt")
//     {
//     }
//
//     PlannerResult plan(const SceneConstPtr &scene, const robot_state::RobotStatePtr &start_state)
//     {
//         // This is required by any TrajOptPlanner::plan() method.
//         ref_state_ = std::make_shared<robot_state::RobotState>(*start_state);
//
//         // Transform robowflex scene to tesseract environment.
//         hypercube::sceneToTesseractEnv(scene, env_);
//
//         // Fill in the problem construction info and initialization.
//         auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(env_);
//         problemConstructionInfo(pci);
//
//         // Add velocity cost.
//         addVelocityCost(pci);
//
//         // Add start state.
//         addStartState(start_state, pci);
//
//         // Add cartesian terms in constraints_.
//         addCartTerms(pci);
//
//         return solve(scene, pci);
//     }
//
//     std::vector<CartCnt> constraints_{};
//
// private:
//     void addCartTerms(std::shared_ptr<trajopt::ProblemConstructionInfo> pci) const
//     {
//         for (const auto &term : constraints_)
//         {
//             Eigen::Quaterniond rotation(term.pose.linear());
//             auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
//             pose_constraint->term_type = trajopt::TT_CNT;
//             pose_constraint->link = term.link;
//             pose_constraint->timestep = term.timestep;
//             pose_constraint->xyz = term.pose.translation();
//             pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
//             pose_constraint->pos_coeffs = Eigen::Vector3d::Constant(term.pos_coeffs);
//             pose_constraint->rot_coeffs = Eigen::Vector3d::Constant(term.rot_coeffs);
//             pose_constraint->name = "pose_cnt_link_" + term.link + std::to_string(term.timestep);
//
//             pci->cnt_infos.push_back(pose_constraint);
//         }
//     }
// };


int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    bool enable_mobile_base = false;
    if (GROUP == "mobile_base_manipulator")
        enable_mobile_base = true;

    // Create the default Fetch robot.
    auto stretch = std::make_shared<StretchRobot>();
    stretch->initialize(false, enable_mobile_base, GROUP, "stretch_head");
    stretch->setState(std::map<std::string, double>{{"joint_lift", 0.2}});
    auto const &model = stretch->getModelConst();
    auto const &gmodel = model->getJointModelGroup(GROUP);
    auto names = gmodel->getJointModelNames();
    for (auto const &name : names)
        std::cout << name << std::endl;

    // stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 1.0});
    // stretch->setGroupState(GROUP, {0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
    stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0.0, 0.0});
    // stretch->setGroupState(GROUP, {0.0, 0.0});

    // Load tabletop scene.
    auto scene = std::make_shared<Scene>(stretch);
    scene->fromYAMLFile("package://robowflex_tesseract/scenes/table/scene.yaml");
    scene->getCurrentState() = *stretch->getScratchState();

    // auto robot_offset = TF::createPoseQ(Eigen::Vector3d{0.5, 0.0, 0}, Eigen::Quaterniond{0.707, 0, 0, -0.707});//-90 z
    // auto robot_offset = TF::createPoseQ(Eigen::Vector3d{-0.5, 0.0, 0}, Eigen::Quaterniond{1,0,0,0});
    // scene->moveAllObjectsGlobal(robot_offset);

    // RVIZ helper.
    const auto &rviz = std::make_shared<IO::RVIZHelper>(stretch);
    rviz->updateScene(scene);

    RBX_INFO("Press Enter to continue");


    // Create the default planner for the Fetch.
    auto sbmp_planner = std::make_shared<OMPL::StretchOMPLPipelinePlanner>(stretch, "default");
    sbmp_planner->initialize();
    const auto &request = std::make_shared<MotionRequestBuilder>(stretch, GROUP);
    request->setStartConfiguration(stretch->getScratchState());
    stretch->setGroupState(GROUP, {0.0, -0.2, 0.0, 0, 0});
    request->setGoalConfiguration(stretch->getScratchState());
    const auto &sbmp_res = sbmp_planner->plan(scene, request->getRequest());
    if (sbmp_res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        RBX_ERROR("Trajectory not found. Aborting!");
        return 1;
    }
    Trajectory sbmp_trajectory(sbmp_res.trajectory_);


    // Create a TrajOpt planner for Setch.
    // auto planner = std::make_shared<CustomTrajOptPlanner>(stretch, GROUP);
    auto planner = std::make_shared<TrajOptPlanner>(stretch, GROUP);
    // planner->initialize("link_mast", "link_wrist_yaw", enable_mobile_base);
    planner->initialize("link_mast", "link_head_tilt", enable_mobile_base);

    // Set planner parameters.
    int num_waypoints = 10;
    planner->options.num_waypoints = num_waypoints;  // Select number of waypoints in trajectory
    // planner->setInitType(trajopt::InitInfo::Type::JOINT_INTERPOLATED);  // Initialize using a straight-line
                                                                        // between start and goal in C-space.
    // planner->setInitialTrajectory(sbmp_trajectory.getTrajectory());
    planner->options.joint_vel_coeffs = 20.0;
    // planner->options.pose_cnt_pos_coeffs = 60.0;
    // planner->options.pose_cnt_rot_coeffs = 60.0;
    // planner->options.merit_error_coeff = 0.5;
    planner->options.return_first_sol = false;

    std::cout << "links:" << std::endl;
    auto const &manip_links = planner->getManipulatorLinks();
    for (const auto &link : manip_links)
        std::cout << link << std::endl;
    std::cout << "" << std::endl;
    std::cout << "joints:" << std::endl;
    auto const &manip_joints = planner->getManipulatorJoints();
    for (const auto &joint : manip_joints)
        std::cout << joint << std::endl;

    // Get pose of object of interest.
    auto point_pose = scene->getObjectPose("Can1");

    // Load request.
    // const auto &request = std::make_shared<MotionRequestBuilder>(stretch, GROUP);
    stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0, 0});
    request->setStartConfiguration(stretch->getScratchState());

    stretch->setGroupState(GROUP, {0.2, -0.8, 0.0, 0.0, 0.0});
    const RobotPose point_pan = stretch->getLinkTF("link_head_pan").inverse() * point_pose;
    const RobotPose point_tilt = stretch->getLinkTF("link_head_tilt").inverse() * point_pose;

    const double pan = atan2(point_pan.translation().y(), point_pan.translation().x());
    const double tilt = atan2(point_tilt.translation().z(),
                               hypot(point_tilt.translation().x(), point_tilt.translation().y()));

    // const auto &ee_pose = stretch->getLinkTF("link_head_tilt");
    // auto link_pose = TF::createPoseXYZ(ee_pose.translation()[0], ee_pose.translation()[1], ee_pose.translation()[2], 1.57, pan, tilt);

    // stretch->setGroupState(GROUP, {0.5, 0.5, 1.57, 0.9, 0.1, 0.1, 0.1, 0.1, 1.0});
    // stretch->setGroupState(GROUP, {0.9, 0.1, 0.1, 0.1, 0.1, 1.0});
    stretch->setGroupState(GROUP, {0.2, -0.8, 0.0, pan, tilt});
    // stretch->setGroupState(GROUP, {pan, tilt});
    request->setGoalConfiguration(stretch->getScratchState());

    // CustomTrajOptPlanner::CartCnt term;
    // // link_pose.translate({0.0, 0.3, 0.0});
    // term.pose = link_pose;
    // term.timestep = num_waypoints - 1;
    // term.link = "link_head_tilt";
    // term.pos_coeffs = 0.0;
    // term.rot_coeffs = 1.0;
    // planner->constraints_.push_back(term);

    std::cin.ignore();

    scene->getCurrentState() = *stretch->getScratchState();

    // Do motion planning.
    const auto &res = planner->plan(scene, request->getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        RBX_ERROR("Trajectory not found. Aborting!");
        return 1;
    }
    Trajectory trajectory(res.trajectory_);
    // stretch->setGroupState(GROUP, {0.0, 0.0, 0.0, 0.0, 0.0});
    // const auto &res = planner->plan(scene, stretch->getScratchState(), link_pose, "link_head_tilt");
    // if (!res.first)
    // {
    //     RBX_INFO("Optimization did not converge");
    //     return 0;
    // }
    // //
    // // // Publish the trajectory to a topic to display in RViz
    // Trajectory trajectory(planner->getTrajectory());
    rviz->updateTrajectory(trajectory);
    stretch->setState(trajectory.getFinalPositions());
    scene->getCurrentState() = *stretch->getScratchState();
    rviz->updateScene(scene);

    RBX_INFO("Press Enter to exit");
    std::cin.ignore();

    // Clean up RViz.
    // rviz->removeMarker("goal");
    // rviz->updateMarkers();
    rviz->removeScene();

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
