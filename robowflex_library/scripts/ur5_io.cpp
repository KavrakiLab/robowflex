#include <robowflex_library/robowflex.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    startROS(argc, argv);

    Robot ur5("ur5");
    ur5.initialize("package://ur_description/urdf/ur5_robotiq_robot_limited.urdf.xacro",  // urdf
                   "package://ur5_robotiq85_moveit_config/config/ur5_robotiq85.srdf",     // srdf
                   "package://ur5_robotiq85_moveit_config/config/joint_limits.yaml",      // joint limits
                   "package://ur5_robotiq85_moveit_config/config/kinematics.yaml"         // kinematics
    );

    Scene scene(ur5);
    scene.fromYAMLFile("package://robowflex_library/yaml/test.yml");

    {
        IO::Bag bag_out("scene.bag", IO::Bag::WRITE);
        bag_out.addMessage("scene", scene.getMessage());
    }

    {
        IO::Bag bag_in("scene.bag", IO::Bag::READ);
        auto msgs = bag_in.getMessages<moveit_msgs::PlanningScene>({"scene"});
    }

    return 0;
}
