#ifndef ROBOWFLEX_
#define ROBOWFLEX_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/PlanningScene.h>

namespace robowflex
{
    namespace IO
    {
        // Resolves `package://` URLs and returns canonical absolute path if path exists, otherwise ""
        const std::string resolvePath(const std::string &path);

        // Loads an XML (or xacro) file to a string. If path does not exist or bad format, ""
        const std::string loadXMLToString(const std::string &path);

        // Loads a xacro file to a string. If path does not exist or bad format, ""
        const std::string loadXacroToString(const std::string &path);

        // Loads a file to a string. If path does not exist or bad format, ""
        const std::string loadFileToString(const std::string &path);

        // Runs a command and grabs stdout to a string., If fail ""
        const std::string runCommand(const std::string &cmd);

        // Loads an YAML file to a YAML node. If path does not exist or bad format, false in first.
        const std::pair<bool, YAML::Node> loadFileToYAML(const std::string &path);

        class Handler
        {
        public:
            Handler(const std::string &name);

            Handler(Handler const &) = delete;
            void operator=(Handler const &) = delete;

            ~Handler();

            // Loads an YAML node to the ROS parameter server.
            void loadYAMLtoROS(const YAML::Node &node, const std::string &prefix);

            template <typename T>
            void setParam(const std::string &key, const T &value)
            {
                nh_.setParam(key, value);
                params_.emplace_back(key);
            }

            bool hasParam(const std::string &key)
            {
                return nh_.hasParam(key);
            }

            template <typename T>
            bool getParam(const std::string &key, const T &value)
            {
                return nh_.getParam(key, value);
            }

            const ros::NodeHandle &getHandle()
            {
                return nh_;
            }

            const std::string &getNamespace()
            {
                return namespace_;
            }

            template <typename T>
            ros::Publisher advertise(const std::string &name)
            {
                return nh_.advertise<T>(name, 1000);
            }

        private:
            // Generates a UUID
            static const std::string generateUUID();
            static const std::string UUID;

            const std::string name_;
            const std::string namespace_;
            ros::NodeHandle nh_;

            std::vector<std::string> params_;
        };
    }  // namespace IO

    namespace TF
    {
        Eigen::Vector3d vectorMsgToEigen(const geometry_msgs::Vector3 &msg);
        geometry_msgs::Vector3 vectorEigenToMsg(const Eigen::Vector3d &vector);
        Eigen::Affine3d poseMsgToEigen(const geometry_msgs::Pose &msg);
        geometry_msgs::Pose poseEigenToMsg(const Eigen::Affine3d &pose);
    }  // namespace TF

    class Geometry
    {
    public:
        class ShapeType
        {
        public:
            enum Type
            {
                BOX = 0,
                SPHERE = 1,
                CYLINDER = 2,
                CONE = 3,
                MESH = 4
            };

            static const unsigned int MAX;
            static const std::vector<std::string> STRINGS;

            static Type toType(const std::string &str);
            static const std::string &toString(Type type);
        };

        Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource = "",
                 const Eigen::Affine3d &offset = Eigen::Affine3d::Identity());

        Geometry(const Geometry &) = delete;             // non construction-copyable
        Geometry &operator=(const Geometry &) = delete;  // non copyable

        const bool isMesh() const;

        const shape_msgs::SolidPrimitive getSolidMsg() const;
        const shape_msgs::Mesh getMeshMsg() const;

    private:
        std::shared_ptr<shapes::Shape> loadShape() const;

        ShapeType::Type type_{ShapeType::Type::BOX};                 // Geometry Type.
        std::string resource_{""};                                   // Resource locator for MESH types.
        const Eigen::Vector3d dimensions_{Eigen::Vector3d::Ones()};  // Dimensions to scale geometry along axes.
        const Eigen::Affine3d offset_;                               // Offset of geometry from base frame.
        const std::shared_ptr<shapes::Shape> shape_{nullptr};        // Loaded mesh.
    };

    class Robot
    {
    public:
        Robot(const std::string &name);

        Robot(Robot const &) = delete;
        void operator=(Robot const &) = delete;

        bool initialize(const std::string &urdf_file, const std::string &srdf_file, const std::string &limits_file,
                        const std::string &kinematics_file);

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

    protected:
        // Loads a robot description (URDF, SRDF, joint limits, kinematics) to the parameter server under
        // "description".
        // Returns false when failure.
        bool loadRobotDescription(const std::string &urdf_file, const std::string &srdf_file,
                                  const std::string &limits_file, const std::string &kinematics_file);
        void loadRobotModel();

        const std::string name_;
        IO::Handler handler_;

        robot_model::RobotModelPtr model_;

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

        Scene(Scene const &) = delete;
        void operator=(Scene const &) = delete;

        planning_scene::PlanningScenePtr &getScene()
        {
            scene_->setPlanningSceneMsg(msg_);
            return scene_;
        }

        moveit_msgs::PlanningScene &getMessage()
        {
            return msg_;
        }

        robot_state::RobotState &getCurrentState();

        collision_detection::AllowedCollisionMatrix getACM()
        {
            return collision_detection::AllowedCollisionMatrix(msg_.allowed_collision_matrix);
        }

        void setACM(const collision_detection::AllowedCollisionMatrix &acm)
        {
            moveit_msgs::AllowedCollisionMatrix acm_msg;
            acm.getMessage(acm_msg);

            msg_.allowed_collision_matrix = acm_msg;
        }

        void addCollisionObject(const std::string &name, const Geometry &geometry, const std::string &base_frame,
                                const Eigen::Affine3d &pose);

        void removeCollisionObject(const std::string &name);

    private:
        planning_scene::PlanningScenePtr scene_;
        moveit_msgs::PlanningScene msg_;
    };

    class Planner
    {
    public:
        Planner(Robot &robot) : robot_(robot), handler_(robot_.getHandler())
        {
        }

        Planner(Planner const &) = delete;
        void operator=(Planner const &) = delete;

        virtual planning_interface::MotionPlanResponse plan(Scene &scene,
                                                            const planning_interface::MotionPlanRequest &request) = 0;

    protected:
        Robot &robot_;
        IO::Handler &handler_;
    };

    class PipelinePlanner : public Planner
    {
    public:
        PipelinePlanner(Robot &robot) : Planner(robot)
        {
        }

        PipelinePlanner(PipelinePlanner const &) = delete;
        void operator=(PipelinePlanner const &) = delete;

        planning_interface::MotionPlanResponse plan(Scene &scene,
                                                    const planning_interface::MotionPlanRequest &request) override;

    protected:
        planning_pipeline::PlanningPipelinePtr pipeline_;
    };

    class OMPLPlanner : public PipelinePlanner
    {
    public:
        class OMPLSettings
        {
        public:
            // Initialized here so default arguments are parsed correctly in loadOMPLPipeline.
            OMPLSettings()
              : max_goal_samples(10)
              , max_goal_sampling_attempts(1000)
              , max_planning_threads(4)
              , max_solution_segment_length(0.0)
              , max_state_sampling_attempts(4)
              , minimum_waypoint_count(10)
              , simplify_solutions(true)
              , use_constraints_approximations(false)
              , display_random_valid_states(false)
              , link_for_exploration_tree("")
              , maximum_waypoint_distance(0.0)
            {
            }

            int max_goal_samples;
            int max_goal_sampling_attempts;
            int max_planning_threads;
            double max_solution_segment_length;
            int max_state_sampling_attempts;
            int minimum_waypoint_count;
            bool simplify_solutions;
            bool use_constraints_approximations;
            bool display_random_valid_states;
            std::string link_for_exploration_tree;
            double maximum_waypoint_distance;

            void setParam(IO::Handler &handler) const;
        };

        OMPLPlanner(Robot &robot);

        OMPLPlanner(OMPLPlanner const &) = delete;
        void operator=(OMPLPlanner const &) = delete;

        bool initialize(const std::string &config_file = "", const OMPLSettings settings = OMPLSettings(),
                        const std::string &plugin = "ompl_interface/OMPLPlanner",
                        const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

    private:
        static const std::vector<std::string> DEFAULT_ADAPTERS;
    };

    class MotionRequestBuilder
    {
    public:
        MotionRequestBuilder(const Robot &robot, const std::string &group_name);

        void setStartConfiguration(const std::vector<double> &joints);
        void setGoalConfiguration(const std::vector<double> &joints);
        void setGoalConfiguration(const geometry_msgs::PoseStamped goal_pose, std::string ee_name);
        const planning_interface::MotionPlanRequest &getRequest();

    private:
        const Robot &robot_;
        const std::string group_name_;
        const robot_model::JointModelGroup *jmg_;

        planning_interface::MotionPlanRequest request_;
    };

    class RVIZHelper
    {
    public:
        RVIZHelper(Robot &robot, Scene &scene) : robot_(robot), scene_(scene)
        {
            IO::Handler &handler = robot.getHandler();

            traj_pub_ = handler.advertise<moveit_msgs::RobotTrajectory>("trajectory");
            scene_pub_ = handler.advertise<moveit_msgs::PlanningScene>("scene");
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

}  // namespace robowflex

#endif
