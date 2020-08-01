/* Author: Carlos Quintero */

// ROS
#include <moveit_msgs/MoveItErrorCodes.h>

// Robowflex
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/yaml.h>
#include <robowflex_ompl/ompl_interface.h>
#include <robowflex_tesseract/trajopt_planner.h>

using namespace robowflex;
int main(int argc, char** argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "tesseract_test");
    ros::NodeHandle node("~");

    // Parameters
    std::string ompl_config = ros::package::getPath("fetch_moveit_config")+"/config/ompl_planning.yaml";
    std::string planning_group = "arm";
    std::string dataset = ros::package::getPath("robowflex_tesseract")+"/scenes/table";
    std::string manipulator = "arm_chain";
    int start = 1;
    int end = 5;
    int num_waypoints = 20;
    bool solve = true;
    bool file_write_cb = false;
    bool use_ik_solution = true;
    bool use_straight_line_init = true;

    // Fetch robot
    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize(false);
    auto ee = fetch->getModel()->getEndEffectors()[0]->getLinkModelNames()[0];
    auto root_name = fetch->getModelConst()->getRootLinkName();
    
    // OMPL planner
    auto planner = std::make_shared<OMPL::OMPLInterfacePlanner>(fetch, "default");
    planner->initialize(ompl_config);
    
    // RVIZ helper
    auto rviz = std::make_shared<IO::RVIZHelper>(fetch);
    auto tol = Eigen::Vector3d{0.0001, 0.0001, 0.0001};
    auto region = robowflex::Geometry::makeBox(0.01, 0.01, 0.01);
    auto rot = Eigen::Quaterniond{1, 0, 0, 0};
    auto color = Eigen::Vector4d{0.0, 0.0, 1.0, 1.0};
    auto scale = Eigen::Vector3d{0.1, 0.008, 0.008};
    
    // TrajOpt planner
    auto trajopt_planner = std::make_shared<TrajOptPlanner>(fetch, planning_group, manipulator);
    trajopt_planner->setNumWaypoints(num_waypoints);
    trajopt_planner->setWriteFile(file_write_cb, ros::package::getPath("robowflex_tesseract"));

    // Loop over recorded scenes and requests
    for (int i = start; i <= end; i++)
    {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << i;
        std::string index = ss.str();

        // Load scene
        auto fscene = dataset + "/scene_vicon" + index + ".yaml";
        auto scene = std::make_shared<Scene>(fetch);
        if (!scene->fromYAMLFile(fscene))
        {
            ROS_ERROR("Failed to read file: %s for scene", fscene.c_str());
            continue;
        }
        rviz->updateScene(scene);
        
        // Load request
        auto frequest = dataset + "/request" + index + ".yaml";
        auto request = std::make_shared<MotionRequestBuilder>(planner, planning_group);
        if (!request->fromYAMLFile(frequest))
        {
            ROS_ERROR("Failed to read file: %s for request", frequest.c_str());
            continue;
        }
        
        // Add a marker to the pick_pose
        auto pick_state = request->getGoalConfiguration();
        auto pick_ee_pose = pick_state->getFrameTransform(ee);
        auto ndof = pick_state->getVariableCount();
        rviz->addArrowMarker("pick_pose", root_name, pick_ee_pose, color, scale);
        rviz->updateMarkers();
        
        // Solve Pick problem using OMPL planner (or visualize a previously recorded trajectory)
        if (solve)
        {               
            // Visualize goal state
            rviz->visualizeState(pick_state);
            
            // Solve the pick problem using a Moveit-OMPL planner
            auto res = planner->plan(scene, request->getRequest());
            if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
                rviz->updateTrajectory(res);
        }
        else
        {
            // Read solution trajectory
            auto fpath = dataset + "/path" + index + ".yaml";
            moveit_msgs::RobotTrajectory traj;
            if (!IO::YAMLFileToMessage(traj, fpath))
            {
                ROS_ERROR("Failed to read file: %s for path", fpath.c_str());
                continue;
            }
            
            // Visualize goal state
            rviz->visualizeState(pick_state);
            
            // Visualize trajectory
            auto start_state = request->getStartConfiguration();
            rviz->updateTrajectory(traj, *start_state);
        }
        
        ROS_INFO("Visualizing pick state and trajectory");
        ROS_ERROR("Press enter to continue");
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        // Solve Place problem using TrajOpt planner
        
        // Define initial state (as the goal state in the request)
        double* pick_state_dbl = new double((int)ndof);
        pick_state_dbl = pick_state->getVariablePositions();
        std::unordered_map<std::string, double> pick_state_map;
        for (int i=0;i<(int)ndof;i++)
            pick_state_map[pick_state->getVariableNames()[i]] = pick_state_dbl[i];
        
        // Define final end-effector pose
        Eigen::Isometry3d place_ee_pose = pick_ee_pose;
        //place_ee_pose.translation() += Eigen::Vector3d(0.0, 0.65, 0.0);
        place_ee_pose.translation() += Eigen::Vector3d(0.0, -0.6, 0.0);
        
        // Add a marker to the goal_pose
        rviz->addArrowMarker("goal_pose", root_name, place_ee_pose, color, scale);
        rviz->updateMarkers();
        scene->attachObject("Can1");
        
        // Either use start/goal states or a start state and an end-effector goal pose
        if (use_ik_solution)
        {
            // Solve problem using IK query from the goal pose
            std::vector<double> pick_state_vct(pick_state_dbl, pick_state_dbl+(int)ndof);
            fetch->setState(pick_state_vct);
            if (!fetch->setFromIKCollisionAware(scene, planning_group, region, place_ee_pose, rot, tol))
            {
                ROS_ERROR("IK solution not found for the Place problem");
                continue;
            }
            auto place_state = fetch->getScratchState();
            rviz->visualizeState(place_state);
            
            // Initialize trajectory using a straight line in c-space
            if (use_straight_line_init)
                trajopt_planner->setInitType(trajopt::InitInfo::Type::JOINT_INTERPOLATED);
            
            // Solve the problem for an initial and a goal state
            if (trajopt_planner->plan(scene, pick_state, place_state))
                rviz->updateTrajectory(trajopt_planner->getTrajectory());
        }
        else
        {
            // Solve the problem using goal pose of the end effector
            if (trajopt_planner->plan(scene, pick_state_map, place_ee_pose, ee))
                rviz->updateTrajectory(trajopt_planner->getTrajectory());
        }
        
        ROS_INFO("Visualizing place state and trajectory");
        ROS_ERROR("Press enter to continue");
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return 0;
}
