#ifndef ROBOWFLEX_SCENE_
#define ROBOWFLEX_SCENE_

namespace robowflex
{
    class Robot
    {
    public:
        Robot(const std::string &name);

        Robot(Robot const &) = delete;
        void operator=(Robot const &) = delete;

        bool initialize(const std::string &urdf_file, const std::string &srdf_file, const std::string &limits_file,
                        const std::string &kinematics_file);

        bool loadYAMLFile(const std::string &name, const std::string &file);
        bool loadXMLFile(const std::string &name, const std::string &file);

        const std::string &getName() const
        {
            return name_;
        }

        const robot_model::RobotModelPtr &getModel() const
        {
            return model_;
        }

        robot_model::RobotModelPtr &getModel()
        {
            return model_;
        }

        IO::Handler &getHandler()
        {
            return handler_;
        }

        bool loadKinematics(const std::string &group);

    protected:
        // Loads a robot description (URDF, SRDF, joint limits, kinematics) to the parameter server
        // Returns false when failure.
        bool loadRobotDescription(const std::string &urdf_file, const std::string &srdf_file,
                                  const std::string &limits_file, const std::string &kinematics_file);
        void loadRobotModel();

        const std::string name_;
        IO::Handler handler_;

        std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;
        robot_model::RobotModelPtr model_;
        std::map<std::string, robot_model::SolverAllocatorFn> imap_;
        kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_;

    private:
        static const std::string ROBOT_DESCRIPTION;
        static const std::string ROBOT_SEMANTIC;
        static const std::string ROBOT_PLANNING;
        static const std::string ROBOT_KINEMATICS;
    };

    class Scene
    {
    public:
        Scene(Robot &robot);

        Scene(const Scene &);
        void operator=(const Scene &);

        const planning_scene::PlanningScenePtr &getSceneConst() const
        {
            return scene_;
        }

        planning_scene::PlanningScenePtr &getScene()
        {
            return scene_;
        }

        moveit_msgs::PlanningScene getMessage() const;
        robot_state::RobotState &getCurrentState();
        collision_detection::AllowedCollisionMatrix &getACM();

        void updateCollisionObject(const std::string &name, const Geometry &geometry, const Eigen::Affine3d &pose);
        void removeCollisionObject(const std::string &name);
        Eigen::Affine3d getObjectPose(const std::string &name);

        // Use default end-effector if one exists
        bool attachObject(const std::string &name);
        bool attachObject(const std::string &name, const std::string &ee_link,
                          const std::vector<std::string> &touch_links);
        bool detachObject(const std::string &name);

        bool toYAMLFile(const std::string &file);
        bool fromYAMLFile(const std::string &file);
        bool fromOpenRAVEXMLFile(const std::string &file);

    private:
        planning_scene::PlanningScenePtr scene_;
    };

    namespace IO
    {
        class RVIZHelper
        {
        public:
            RVIZHelper(Robot &robot, Scene &scene) : robot_(robot), scene_(scene)
            {
                ros::NodeHandle nh("~");

                traj_pub_ = nh.advertise<moveit_msgs::RobotTrajectory>("trajectory", 1000);
                scene_pub_ = nh.advertise<moveit_msgs::PlanningScene>("scene", 1000);
            }

            void update(const planning_interface::MotionPlanResponse &response)
            {
                moveit_msgs::RobotTrajectory msg;
                response.trajectory_->getRobotTrajectoryMsg(msg);

                traj_pub_.publish(msg);
                scene_pub_.publish(scene_.getMessage());
            }

        private:
            Robot &robot_;
            Scene &scene_;

            ros::Publisher traj_pub_;
            ros::Publisher scene_pub_;
        };
    }  // namespace IO
}  // namespace robowflex

#endif
