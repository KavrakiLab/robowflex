/* Author: Carlos Quintero */

// Robowflex
#include <robowflex_library/util.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/yaml.h>

// Robowflex tesseract
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    ROS ros(argc, argv, "fetch_tabletop", 0);

    // Parameters.
    const auto &dataset = IO::resolvePackage("package://robowflex_tesseract/scenes/table");
    const auto &planning_group = "arm";
    const auto &manipulator = "arm_chain";
    int start = 1;
    int end = 10;
    int num_waypoints = 15;
    bool solve = true;
    bool file_write_cb = false;
    bool use_goal_state = false;
    bool use_straight_line_init = true;

    // Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);
    const auto &ee = fetch->getModel()->getEndEffectors()[0]->getLinkModelNames()[0];
    const auto &root_name = fetch->getModelConst()->getRootLinkName();

    // RVIZ helper.
    auto rviz = std::make_shared<IO::RVIZHelper>(fetch);
    const auto &color = Eigen::Vector4d{0.0, 0.0, 1.0, 1.0};
    const auto &scale = Eigen::Vector3d{0.1, 0.008, 0.008};

    // TrajOpt planner.
    auto trajopt_planner = std::make_shared<TrajOptPlanner>(fetch, planning_group, manipulator);
    if (!trajopt_planner->initialize())
        return -1;
    trajopt_planner->options.num_waypoints = num_waypoints;
    trajopt_planner->setWriteFile(file_write_cb, dataset);

    // Loop over recorded scenes and requests.
    for (int i = start; i <= end; i++)
    {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;
        std::string index = ss.str();

        // Load scene
        boost::filesystem::path scene_path(dataset);
        scene_path /= "scene_vicon" + index + ".yaml";
        const auto &scene = std::make_shared<Scene>(fetch);
        if (!scene->fromYAMLFile(scene_path.string()))
        {
            ROS_ERROR("Failed to read file: %s for scene", scene_path.string().c_str());
            continue;
        }
        rviz->updateScene(scene);

        // Load place request
        boost::filesystem::path request_path(dataset);
        request_path /= "place_request" + index + ".yaml";
        const auto &place_request = std::make_shared<MotionRequestBuilder>(trajopt_planner, planning_group);
        if (!place_request->fromYAMLFile(request_path.string()))
        {
            ROS_ERROR("Failed to read file: %s for request", request_path.string().c_str());
            continue;
        }

        // Extract place state and ee pose from request.
        const auto &place_state = place_request->getGoalConfiguration();
        const auto &place_ee_pose = place_state->getFrameTransform(ee);

        // Add a marker to the place_pose.
        rviz->addArrowMarker("place_pose", root_name, place_ee_pose, color, scale);
        rviz->updateMarkers();

        // Solve Place problem using TrajOpt planner or visualize a previously recorded trajectory.
        if (solve)
        {
            // Visualize place state.
            rviz->visualizeState(place_state);

            if (use_goal_state)
            {
                // Initialize trajectory using a straight line in c-space.
                if (use_straight_line_init)
                    trajopt_planner->setInitType(trajopt::InitInfo::Type::JOINT_INTERPOLATED);

                // Solve the place problem using TrajOpt with start and goal states.
                const auto &res = trajopt_planner->plan(scene, place_request->getRequest());
                if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                    rviz->updateTrajectory(res);
            }
            else
            {
                // Solve the place problem using TrajOpt with a start state and a goal pose for the end
                // effector.
                const auto &pick_state = place_request->getStartConfiguration();
                if (trajopt_planner->plan(scene, pick_state, place_ee_pose, ee))
                    rviz->updateTrajectory(trajopt_planner->getTrajectory());
            }
        }
        else
        {
            // Read solution trajectory.
            boost::filesystem::path traj_path(dataset);
            traj_path /= "place_path" + index + ".yaml";
            moveit_msgs::RobotTrajectory traj;
            if (!IO::YAMLFileToMessage(traj, traj_path.string()))
            {
                ROS_ERROR("Failed to read file: %s for path", traj_path.string().c_str());
                continue;
            }

            // Visualize place state.
            rviz->visualizeState(place_state);

            // Visualize trajectory.
            const auto &start_state = place_request->getStartConfiguration();
            rviz->updateTrajectory(traj, *start_state);
        }

        ROS_INFO("Visualizing place state and trajectory");
        std::cout << "\033[1;32m<"
                  << "Press Enter to continue"
                  << ">\033[0m. " << std::endl;
        std::cin.ignore();
    }

    ros::spin();
    return 0;
}
