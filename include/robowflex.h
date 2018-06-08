#ifndef ROBOWFLEX_
#define ROBOWFLEX_

#include <yaml-cpp/yaml.h>

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

        private:
            // Generates a UUID
            static const std::string generateUUID();
            static const std::string UUID;

            const std::string name_;
            const std::string namespace_;
            const ros::NodeHandle nh_;

            std::vector<std::string> params_;
        };
    }  // namespace IO

    class Robot
    {
    public:
        Robot(const std::string &name);

        Robot(Robot const &) = delete;
        void operator=(Robot const &) = delete;

        bool initialize(const std::string &urdf_file, const std::string &srdf_file, const std::string &limits_file,
                        const std::string &kinematics_file);

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
        class CollisionObject
        {
        public:
            CollisionObject();

        private:
            moveit_msgs::CollisionObject msg_;
        };

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

}  // namespace robowflex

#endif
