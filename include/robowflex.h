#ifndef ROBOWFLEX_
#define ROBOWFLEX_

#include <yaml-cpp/yaml.h>

#include <moveit/robot_model/robot_model.h>

// #define RBF_CLASS_FORWARD(C)                                                                                           \ //     class C;                                                                                                           \
//     typedef std::shared_ptr<C> C##Ptr

namespace robowflex
{
    // Resolves `package://` URLs and returns canonical absolute path if path exists, otherwise ""
    const std::string resolvePath(const std::string &path);
    const std::string loadFileToXML(const std::string &path);
    const YAML::Node loadFileToYAML(const std::string &path);

    void loadYAMLtoROS(const YAML::Node &node, const std::string &prefix);

    void loadRobotDescription(const std::string &description, const std::string &urdf_file,
                              const std::string &srdf_file, const std::string &limits_file,
                              const std::string &kinematics_file);

    robot_model::RobotModelPtr loadRobotModel(const std::string &description);

}  // namespace robowflex

#endif
