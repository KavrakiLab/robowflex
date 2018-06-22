#ifndef ROBOWFLEX_SCENE_
#define ROBOWFLEX_SCENE_

namespace robowflex
{
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);

    class Robot
    {
    public:
        Robot(const std::string &name);

        Robot(Robot const &) = delete;
        void operator=(Robot const &) = delete;

        bool initialize(const std::string &urdf_file, const std::string &srdf_file,
                        const std::string &limits_file, const std::string &kinematics_file);

        bool loadYAMLFile(const std::string &name, const std::string &file);
        bool loadXMLFile(const std::string &name, const std::string &file);

        const std::string &getName() const
        {
            return name_;
        }

        const robot_model::RobotModelPtr &getModelConst() const
        {
            return model_;
        }

        robot_model::RobotModelPtr &getModel()
        {
            return model_;
        }

        robot_model::RobotStatePtr &getScratchState()
        {
            return scratch_;
        }

        IO::Handler &getHandler()
        {
            return handler_;
        }

        bool loadKinematics(const std::string &group);

        void setState(const std::vector<double> &positions);
        void setState(const std::map<std::string, double> &variable_map);
        void setState(const std::vector<std::string> &variable_names,
                      const std::vector<double> &variable_position);

        void setGroupState(const std::string &name, const std::vector<double> &positions);
        std::vector<double> getState() const;
        std::vector<std::string> getJointNames() const;

        void setFromIK(const std::string &group, const Geometry &region, const Eigen::Affine3d &pose,
                       const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);

        const Eigen::Affine3d &getLinkTF(const std::string &name) const;
        bool inCollision(const SceneConstPtr &scene) const;

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

        robot_state::RobotStatePtr scratch_;

    private:
        static const std::string ROBOT_DESCRIPTION;
        static const std::string ROBOT_SEMANTIC;
        static const std::string ROBOT_PLANNING;
        static const std::string ROBOT_KINEMATICS;
    };

    class Scene
    {
    public:
        Scene(const RobotConstPtr &robot);

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

        void updateCollisionObject(const std::string &name, const Geometry &geometry,
                                   const Eigen::Affine3d &pose);
        void removeCollisionObject(const std::string &name);
        Eigen::Affine3d getObjectPose(const std::string &name);

        // Use default end-effector if one exists
        bool attachObject(const std::string &name);
        bool attachObject(const std::string &name, const std::string &ee_link,
                          const std::vector<std::string> &touch_links);
        bool detachObject(const std::string &name);

        bool toYAMLFile(const std::string &file);
        bool fromYAMLFile(const std::string &file);

    private:
        planning_scene::PlanningScenePtr scene_;
    };

    namespace IO
    {
        class RVIZHelper
        {
        public:
            RVIZHelper():
            nh_("~")
            {
                trajectory_pub_ = nh_.advertise<moveit_msgs::RobotTrajectory>("trajectory", 0);
                scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("scene", 0);
                marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 0);
            }

            void updateTrajectory(const planning_interface::MotionPlanResponse &response)
            {
                moveit_msgs::RobotTrajectory msg;
                response.trajectory_->getRobotTrajectoryMsg(msg);

                trajectory_pub_.publish(msg);
            }

            void updateScene(const Scene &scene)
            {
                scene_pub_.publish(scene.getMessage());
            }

            void updateMarkers()
            {
                visualization_msgs::MarkerArray msg;

                std::vector<std::string> remove;
                for (auto &marker : markers_)
                {
                    msg.markers.push_back(marker.second);

                    if (marker.second.action == visualization_msgs::Marker::ADD)
                        marker.second.action = visualization_msgs::Marker::MODIFY;
                    else if (marker.second.action == visualization_msgs::Marker::DELETE)
                        remove.push_back(marker.first);
                }

                marker_pub_.publish(msg);

                for (auto &marker : remove)
                    markers_.erase(markers_.find(marker));
            }

        private:
            ros::NodeHandle nh_;
            ros::Publisher marker_pub_, trajectory_pub_, scene_pub_;

            std::map<std::string, visualization_msgs::Marker> markers_;
        };
    }  // namespace IO
}  // namespace robowflex

#endif
