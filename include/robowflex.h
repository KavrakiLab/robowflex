#ifndef ROBOWFLEX_
#define ROBOWFLEX_

#include <yaml-cpp/yaml.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>

namespace robowflex
{
    // Resolves `package://` URLs and returns canonical absolute path if path exists, otherwise ""
    const std::string resolvePath(const std::string &path);

    // Loads an XML (or xacro) file to a string. If path does not exist or bad format, ""
    const std::string loadFileToXML(const std::string &path);

    // Loads an YAML file to a YAML node. If path does not exist or bad format, false in first.
    const std::pair<bool, YAML::Node> loadFileToYAML(const std::string &path);

    // Loads an YAML node to the ROS parameter server.
    void loadYAMLtoROS(const YAML::Node &node, const std::string &prefix, const ros::NodeHandle &nh);

    class Robot
    {
    public:
        Robot(const std::string &name);
        bool initialize(const std::string &urdf_file, const std::string &srdf_file, const std::string &limits_file,
                        const std::string &kinematics_file);

        void loadOMPLPipeline(const std::string &config_file, const std::string &plugin = "ompl_interface/OMPLPlanner",
                              const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

    protected:
        // Loads a robot description (URDF, SRDF, joint limits, kinematics) to the parameter server under
        // "description".
        // Returns false when failure.
        bool loadRobotDescription(const std::string &urdf_file, const std::string &srdf_file,
                                  const std::string &limits_file, const std::string &kinematics_file);
        robot_model::RobotModelPtr loadRobotModel();

        const std::string name_;
        ros::NodeHandle nh_;

        robot_model::RobotModelPtr model_;
        planning_scene::PlanningScenePtr scene_;
        planning_pipeline::PlanningPipelinePtr pipeline_;

    private:
        static const std::vector<std::string> DEFAULT_ADAPTERS;
    };

}  // namespace robowflex

#endif
