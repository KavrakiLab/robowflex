/* Author: Carlos Quintero */

// Robowflex
#include <robowflex_library/log.h>
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
    ROS ros(argc, argv, "fetch_tabletop");

    // Parameters.
    const auto &dataset = IO::resolvePackage("package://robowflex_tesseract/scenes/table");
    const auto &moveit_planning_group = "arm";
    const auto &manip = "arm_chain";
    const auto &manip_base_link = "torso_lift_link";
    const auto &manip_tip_link = "l_gripper_finger_link";
    int start = 1;
    int end = 20;
    int num_waypoints = 10;
    double trajopt_iterations_limit = 100.0;
    bool solve = true;
    bool file_write_cb = false;
    bool use_goal_state = true;
    bool use_straight_line_init = true;

    // Fetch robot.
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);
    const auto &ee = fetch->getModel()->getEndEffectors()[0]->getLinkModelNames()[0];

    // RVIZ helper.
    auto rviz = std::make_shared<IO::RVIZHelper>(fetch);

    // TrajOpt planner.
    auto trajopt_planner = std::make_shared<TrajOptPlanner>(fetch, moveit_planning_group);

    // Initialize planner for a new grop arm_chain with all links from torso_lift_link to gripper_link.
    if (!trajopt_planner->initialize(manip, manip_base_link, manip_tip_link))
        return -1;

    // Set planner parameters.
    trajopt_planner->options.num_waypoints = num_waypoints;
    trajopt_planner->options.max_iter = trajopt_iterations_limit;
    trajopt_planner->setWriteFile(file_write_cb, dataset);

    // Loop over recorded scenes and requests.
    for (int i = start; i <= end; i++)
    {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;
        std::string index = ss.str();

        // Load scene
        boost::filesystem::path scene_path(dataset);
        scene_path /= "scene_table" + index + ".yaml";
        const auto &scene = std::make_shared<Scene>(fetch);
        if (!scene->fromYAMLFile(scene_path.string()))
        {
            RBX_ERROR("Failed to read file: %s for scene", scene_path.string());
            continue;
        }
        rviz->updateScene(scene);

        // Load place request
        boost::filesystem::path request_path(dataset);
        request_path /= "request_table" + index + ".yaml";
        const auto &place_request =
            std::make_shared<MotionRequestBuilder>(trajopt_planner, moveit_planning_group);
        if (!place_request->fromYAMLFile(request_path.string()))
        {
            RBX_ERROR("Failed to read file: %s for request", request_path.string());
            continue;
        }

        // Extract place state and ee pose from request.
        const auto &place_state = place_request->getGoalConfiguration();
        const auto &place_ee_pose = place_state->getFrameTransform(ee);

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
                auto result = trajopt_planner->plan(scene, pick_state, place_ee_pose, ee);
                if (result.first)
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
                RBX_ERROR("Failed to read file: %s for path", traj_path.string());
                continue;
            }

            // Visualize place state.
            rviz->visualizeState(place_state);

            // Visualize trajectory.
            const auto &start_state = place_request->getStartConfiguration();
            rviz->updateTrajectory(traj, *start_state);
        }

        RBX_INFO("Visualizing place state and trajectory");
        std::cout << "\033[1;32m<"
                  << "Press Enter to continue"
                  << ">\033[0m. " << std::endl;
        std::cin.ignore();
    }

    RBX_INFO("Finished");

    ros.wait();
    return 0;
}
