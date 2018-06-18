#include <robowflex/robowflex.h>
#include <robowflex/detail/r2.h>

using namespace robowflex;

const std::string R2Robot::URDF{"package://r2_description/urdf/r2c6.urdf"};
const std::string R2Robot::SRDF{"package://r2_moveit_config/config/r2.srdf"};
const std::string R2Robot::LIMITS{"package://r2_moveit_config/config/joint_limits.yaml"};
const std::string R2Robot::KINEMATICS{"package://r2_moveit_config/config/kinematics.yaml"};
const std::string R2Robot::CACHED{"package://robot_ragdoll_demos/config"};
const std::vector<std::string> R2Robot::SAMPLERS{"moveit_r2_constraints/MoveItR2ConstraintSamplerAllocator",  //
                                                 "moveit_r2_constraints/MoveItR2PoseSamplerAllocator",        //
                                                 "moveit_r2_constraints/MoveItR2JointConstraintSamplerAllocator"};

const std::string OMPL::R2OMPLPipelinePlanner::CONFIG{"package://r2_moveit_config/config/ompl_planning.yaml"};
const std::string OMPL::R2OMPLPipelinePlanner::PLUGIN{"ompl_interface/OMPLPlanningContextManager"};

R2Robot::R2Robot() : Robot("r2")
{
}

bool R2Robot::initialize(const std::vector<std::string> kinematics)
{
    bool success = Robot::initialize(URDF, SRDF, LIMITS, KINEMATICS);
    // success &= loadXMLFile("legs/simplified_robot_description",  //
    //                        "package://r2_simplified_urdf/r2c6_legs_only_creepy.xacro");

    // These need to go in the node namespace
    ros::NodeHandle nh("~");
    nh.setParam("cached_ik_path", IO::resolvePath(CACHED));

    std::stringstream ss;
    for (std::size_t i = 0; i < SAMPLERS.size(); ++i)
    {
        ss << SAMPLERS[i];
        if (i < SAMPLERS.size() - 1)
            ss << " ";
    }

    nh.setParam("constraint_samplers", ss.str());

    for (auto &group : kinematics)
        loadKinematics(group);

    return success;
}

OMPL::R2OMPLPipelinePlanner::R2OMPLPipelinePlanner(R2Robot &robot) : OMPLPipelinePlanner(robot)
{
}

bool OMPL::R2OMPLPipelinePlanner::initialize(const std::string &config_file, const Settings settings,
                                             const std::string &plugin, const std::vector<std::string> &adapters)
{
    return OMPLPipelinePlanner::initialize(config_file, settings, plugin, adapters);
}
