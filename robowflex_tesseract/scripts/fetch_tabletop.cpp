/* Author: Carlos Quintero */

// ROS
#include <moveit_msgs/MoveItErrorCodes.h>

// Robowflex
#include <robowflex_library/util.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/yaml.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;
int main(int argc, char **argv)
{
    ROS ros(argc, argv, "fetch_tabletop", 0);
    
    // Parameters
    const auto &dataset = IO::resolvePackage("package://robowflex_tesseract/scenes/table/");
    const auto &planning_group = "arm";
    const auto &manipulator = "arm_chain";
    int start = 1;
    int end = 5;
    int num_waypoints = 15;
    bool solve = true;
    bool file_write_cb = false;
    bool use_goal_state = true;
    bool use_straight_line_init = true;

    // Fetch robot
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);
    const auto &ee = fetch->getModel()->getEndEffectors()[0]->getLinkModelNames()[0];
    const auto &root_name = fetch->getModelConst()->getRootLinkName();

    // RVIZ helper
    auto rviz = std::make_shared<IO::RVIZHelper>(fetch);
    const auto &color = Eigen::Vector4d{0.0, 0.0, 1.0, 1.0};
    const auto &scale = Eigen::Vector3d{0.1, 0.008, 0.008};

    // TrajOpt planner
    auto trajopt_planner = std::make_shared<TrajOptPlanner>(fetch, "default",  planning_group, manipulator);
    if (!trajopt_planner->initialize())
        return -1;
    trajopt_planner->options.num_waypoints = num_waypoints;
    trajopt_planner->setWriteFile(file_write_cb, IO::resolvePackage("package://robowflex_tesseract/scenes/table/"));
    
    // Loop over recorded scenes and requests
    for (int i = start; i <= end; i++)
    {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;
        std::string index = ss.str();

        // Load scene
        const auto &fscene = dataset + "/scene_vicon" + index + ".yaml";
        const auto &scene = std::make_shared<Scene>(fetch);
        if (!scene->fromYAMLFile(fscene))
        {
            ROS_ERROR("Failed to read file: %s for scene", fscene.c_str());
            continue;
        }
        rviz->updateScene(scene);
        
        // Load place request
        const auto &fplace_request = dataset + "/place_request" + index + ".yaml";
        const auto &place_request = std::make_shared<MotionRequestBuilder>(trajopt_planner, planning_group);
        if (!place_request->fromYAMLFile(fplace_request))
        {
            ROS_ERROR("Failed to read file: %s for request", fplace_request.c_str());
            continue;
        }

        // Extract place state and ee pose from request
        const auto &place_state = place_request->getGoalConfiguration();
        const auto &place_ee_pose = place_state->getFrameTransform(ee);
        
        // Add a marker to the place_pose
        rviz->addArrowMarker("place_pose", root_name, place_ee_pose, color, scale);
        rviz->updateMarkers();

        // Solve Place problem using TrajOpt planner (or visualize a previously recorded trajectory)
        if (solve)
        {
            // Visualize place state
            rviz->visualizeState(place_state);

            if (use_goal_state)
            {
                // Initialize trajectory using a straight line in c-space
                if (use_straight_line_init)
                    trajopt_planner->setInitType(trajopt::InitInfo::Type::JOINT_INTERPOLATED);
                // Solve the place problem using TrajOpt
                if (trajopt_planner->plan(scene, place_request))
                    rviz->updateTrajectory(trajopt_planner->getTrajectory());
            }
            else
            {
                // Define initial state
                double *pick_state_dbl = place_request->getStartConfiguration()->getVariablePositions();
                std::unordered_map<std::string, double> pick_state_map;
                for (int i = 0; i < (int)place_request->getStartConfiguration()->getVariableCount(); i++)
                    pick_state_map[place_request->getStartConfiguration()->getVariableNames()[i]] = pick_state_dbl[i];

                // Solve the problem using goal pose of the end effector
                if (trajopt_planner->plan(scene, pick_state_map, place_ee_pose, ee))
                    rviz->updateTrajectory(trajopt_planner->getTrajectory());
            }
        }
        else
        {
            // Read solution trajectory
            const auto &fpath = dataset + "/place_path" + index + ".yaml";
            moveit_msgs::RobotTrajectory traj;
            if (!IO::YAMLFileToMessage(traj, fpath))
            {
                ROS_ERROR("Failed to read file: %s for path", fpath.c_str());
                continue;
            }

            // Visualize place state
            rviz->visualizeState(place_state);

            // Visualize trajectory
            const auto &start_state = place_request->getStartConfiguration();
            rviz->updateTrajectory(traj, *start_state);
        }

        ROS_INFO("Visualizing place state and trajectory");
        std::cout << "\033[1;32m<" << "Press Enter to continue" << ">\033[0m. " << std::endl;
        std::cin.ignore();
    }
    
    ros::spin();
    return 0;
}
