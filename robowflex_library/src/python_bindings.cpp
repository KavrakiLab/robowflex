#include <pybind11/pybind11.h>

#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

#include <memory>
#include <string>
#include <vector>

namespace py = pybind11;
namespace rf = robowflex;

PYBIND11_MODULE(robowflex_library, m)
{
    m.doc() = "Robowflex: MoveIt made easy";

    // Bindings from util.h
    py::class_<rf::Exception>(m, "RobowflexException")
        .def(py::init<int, const std::string &>())
        .def_property_readonly("value", &rf::Exception::getValue)
        .def_property_readonly("message", &rf::Exception::getMessage)
        .def("what", &rf::Exception::what);

    py::class_<rf::ROS>(m, "ROS")
        .def(py::init([](int argc, std::vector<char *> &argv, const std::string &name, unsigned int threads)
                      { return rf::ROS(argc, argv.data(), name, threads); }),
             py::arg("argc"), py::arg("argv"), py::arg("name") = "robowflex", py::arg("threads") = 1)
        .def("get_args", &rf::ROS::getArgs)
        .def("wait", &rf::ROS::wait);

    m.def("explode", &rf::explode);
    // End bindings from util.h

    // Bindings from detail/fetch.h
    py::class_<rf::FetchRobot, rf::FetchRobotPtr>(m, "FetchRobot")
        .def(py::init())
        .def("initialize", &rf::FetchRobot::initialize)
        .def("add_casters_URDF", &rf::FetchRobot::addCastersURDF)
        .def("set_base_pose", &rf::FetchRobot::setBasePose)
        .def("point_head", &rf::FetchRobot::pointHead)
        .def("open_gripper", &rf::FetchRobot::openGripper)
        .def("close_gripper", &rf::FetchRobot::closeGripper);

    using FOPipelinePlanner = rf::OMPL::FetchOMPLPipelinePlanner;
    py::class_<FOPipelinePlanner, std::shared_ptr<FOPipelinePlanner>>(m, "FetchOMPLPipelinePlanner")
        .def(py::init<const rf::RobotPtr &, const std::string &>())
        .def("initialize", &rf::OMPL::FetchOMPLPipelinePlanner::initialize,
             py::arg("settings") = rf::OMPL::Settings(),
             py::arg("adapters") = rf::OMPL::OMPLPipelinePlanner::DEFAULT_ADAPTERS);
    // End bindings from detail/fetch.h

    // Bindings from builder.h
    py::class_<rf::MotionRequestBuilder, rf::MotionRequestBuilderPtr>(m, "MotionRequestBuilder")
        .def(py::init<const rf::RobotConstPtr &>())
        .def(py::init<const rf::RobotConstPtr &, const std::string &, const std::string &>())
        .def(py::init<const rf::PlannerConstPtr &, const std::string &, const std::string &>())
        .def(py::init<const rf::MotionRequestBuilder &>())
        .def("clone", &rf::MotionRequestBuilder::clone)
        .def("initialize", &rf::MotionRequestBuilder::initialize)
        .def("set_planning_group", &rf::MotionRequestBuilder::setPlanningGroup)
        .def("set_planner", &rf::MotionRequestBuilder::setPlanner)
        .def("set_start_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const std::vector<double> &)>(
                 &rf::MotionRequestBuilder::setStartConfiguration))
        .def("set_start_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const robot_state::RobotState &)>(
                 &rf::MotionRequestBuilder::setStartConfiguration))
        .def("set_start_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const robot_state::RobotStatePtr &)>(
                 &rf::MotionRequestBuilder::setStartConfiguration))
        .def("use_scene_state_as_start", &rf::MotionRequestBuilder::useSceneStateAsStart)
        .def("attach_object_to_start", &rf::MotionRequestBuilder::attachObjectToStart)
        .def("attach_object_to_start_const", &rf::MotionRequestBuilder::attachObjectToStartConst)
        .def("add_goal_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const std::vector<double> &)>(
                 &rf::MotionRequestBuilder::addGoalConfiguration))
        .def("add_goal_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const robot_state::RobotStatePtr &)>(
                 &rf::MotionRequestBuilder::addGoalConfiguration))
        .def("add_goal_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const robot_state::RobotState &)>(
                 &rf::MotionRequestBuilder::addGoalConfiguration))
        .def("add_goal_from_IK_query", &rf::MotionRequestBuilder::addGoalFromIKQuery)
        .def("add_goal_pose", &rf::MotionRequestBuilder::addGoalPose)
        .def("add_goal_region", &rf::MotionRequestBuilder::addGoalRegion)
        .def("add_goal_rotary_tile", &rf::MotionRequestBuilder::addGoalRotaryTile)
        .def("add_cylinder_side_grasp", &rf::MotionRequestBuilder::addCylinderSideGrasp)
        .def("set_goal_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const std::vector<double> &)>(
                 &rf::MotionRequestBuilder::setGoalConfiguration))
        .def("set_goal_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const robot_state::RobotStatePtr &)>(
                 &rf::MotionRequestBuilder::setGoalConfiguration))
        .def("set_goal_configuration",
             static_cast<void (rf::MotionRequestBuilder::*)(const robot_state::RobotState &)>(
                 &rf::MotionRequestBuilder::setGoalConfiguration))
        .def("set_goal_from_IK_query", &rf::MotionRequestBuilder::setGoalFromIKQuery)
        .def("set_goal_pose", &rf::MotionRequestBuilder::setGoalPose)
        .def("set_goal_region", &rf::MotionRequestBuilder::setGoalRegion)
        .def("precompute_goal_configurations", &rf::MotionRequestBuilder::precomputeGoalConfigurations)
        .def("clear_goals", &rf::MotionRequestBuilder::clearGoals)
        .def("add_path_pose_constraint", &rf::MotionRequestBuilder::addPathPoseConstraint)
        .def("add_path_position_constraint", &rf::MotionRequestBuilder::addPathPositionConstraint)
        .def("add_path_orientation_constraint", &rf::MotionRequestBuilder::addPathOrientationConstraint)
        .def("set_config", &rf::MotionRequestBuilder::setConfig)
        .def("set_allowed_planning_time", &rf::MotionRequestBuilder::setAllowedPlanningTime)
        .def("set_num_planning_attempts", &rf::MotionRequestBuilder::setNumPlanningAttempts)
        .def("set_workspace_bounds",
             static_cast<void (rf::MotionRequestBuilder::*)(const moveit_msgs::WorkspaceParameters &)>(
                 &rf::MotionRequestBuilder::setWorkspaceBounds))
        .def("set_workspace_bounds",
             static_cast<void (rf::MotionRequestBuilder::*)(const Eigen::Ref<const Eigen::VectorXd> &,
                                                            const Eigen::Ref<const Eigen::VectorXd> &)>(
                 &rf::MotionRequestBuilder::setWorkspaceBounds))
        .def("swap_start_with_goal", &rf::MotionRequestBuilder::swapStartWithGoal)
        .def("get_request", &rf::MotionRequestBuilder::getRequest)
        .def("get_request_const", &rf::MotionRequestBuilder::getRequestConst)
        .def("get_start_configuration", &rf::MotionRequestBuilder::getStartConfiguration)
        .def("get_goal_configuration", &rf::MotionRequestBuilder::getGoalConfiguration)
        .def("get_path_constraints", &rf::MotionRequestBuilder::getPathConstraints)
        .def("get_robot", &rf::MotionRequestBuilder::getRobot)
        .def("get_planner", &rf::MotionRequestBuilder::getPlanner)
        .def("get_planning_group", &rf::MotionRequestBuilder::getPlanningGroup)
        .def("get_planner_config", &rf::MotionRequestBuilder::getPlannerConfig)
        .def("to_yaml_file", &rf::MotionRequestBuilder::toYAMLFile)
        .def("from_yaml_file", &rf::MotionRequestBuilder::fromYAMLFile);
    // End bindings from builder.h

    // TODO: Scene, Robot, MotionPlanResponse, Trajectory
}
