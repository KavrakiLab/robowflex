/* Author: Bryce Willey */

#include <robowflex_tesseract/conversions.h>
#include <robowflex_library/detail/ur5.h>

std::vector<std::vector<double>> NO_COLLISION{
    {0.41, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.95, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.28, -0.99, 1.42, 0.0, 0.0, 0.0},
    {-0.28, -0.44, 0.72, 0.43, 0.0, 0.0}
};

std::vector<std::vector<double>> COLLISION{
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {-0.28, -0.97, 1.42, 0.0, 0.0, 0.0},
    {-0.28, -0.44, 0.72, 0.67, 0.0, 0.0}
};

void test_positions()
{
    // Create a UR5 robot, specifying all necessary files.
    auto ur5_plain = std::make_shared<robowflex::Robot>("ur5");
    ur5_plain->initialize("package://ur_description/urdf/ur5_joint_limited_robot.urdf.xacro", // urdf
                     "package://ur5_moveit_config/config/ur5.srdf",     // srdf
                     "package://ur5_moveit_config/config/joint_limits.yaml",  // joint limits
                     "package://ur5_moveit_config/config/kinematics.yaml"     // kinematics
    );

    // Load kinematics for the WAM7 arm.
    ur5_plain->loadKinematics("manipulator");

    // Create an empty scene.
    auto scene = std::make_shared<robowflex::Scene>(ur5_plain);
    scene->fromYAMLFile("package://robowflex_tesseract/yaml/test_box.yaml");

    tesseract::tesseract_ros::KDLEnvPtr env = robowflex::robow_tesseract::constructTesseractEnv(scene, ur5_plain);

    const auto& manip = env->getManipulator("manipulator");

    // Setting up much of what goes on in the ChainOMPLInterface so we can query specific states.
    std::vector<std::string> joint_names = manip->getJointNames();
    std::vector<std::string> link_names = manip->getLinkNames();
    tesseract::ContactRequest req;
    req.link_names = link_names;
    req.isContactAllowed = [env](const std::string& a, const std::string &b) {
        return env->getAllowedCollisionMatrix()->isCollisionAllowed(a, b);
    };

    // Test the not in contact places.
    for (auto no_col : NO_COLLISION)
    {
        Eigen::Map<Eigen::VectorXd> joint_angles(no_col.data(), 6);
        tesseract::ContactResultMap contact_map;
        env->calcCollisionsDiscrete(req, joint_names, joint_angles, contact_map);
        if (not contact_map.empty())
        {
            ROS_ERROR("Failed! Joints %f, %f, %f, ..., should NOT be in collision, has %d contacts", no_col[0], no_col[1], no_col[2], contact_map.size());

            for (auto links : contact_map)
            {
                ROS_ERROR("Colision between %s and %s:", links.first.first.c_str(), links.first.second.c_str());
                for (auto result : links.second)
                {
                    ROS_ERROR("Contact: distance: %f, nearest_points: %f, %f, %f, and %f, %f, %f", result.distance, 
                            result.nearest_points[0](0), 
                            result.nearest_points[0](1),
                            result.nearest_points[0](2),
                            result.nearest_points[1](0),
                            result.nearest_points[1](1),
                            result.nearest_points[1](2)
                            );
                }
            }
        }
        else
        {
            ROS_INFO("Correct! Joints %f, %f, %f, ..., SHOULD and ARE in collision", no_col[0], no_col[1], no_col[2]);
        }
    }

    // Test the in contact places.
    for (auto col : COLLISION)
    {
        Eigen::Map<Eigen::VectorXd> joint_angles(col.data(), 6);
        tesseract::ContactResultMap contact_map;
        env->calcCollisionsDiscrete(req, joint_names, joint_angles, contact_map);
        if (contact_map.empty())
        {
            ROS_ERROR("Failed! Joints %f, %f, %f, ..., SHOULD be in collision", col[0], col[1], col[2]);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tester");
    test_positions();
    return 0;
}