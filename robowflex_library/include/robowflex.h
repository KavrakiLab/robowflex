#ifndef ROBOWFLEX_
#define ROBOWFLEX_

#include <fstream>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>

// #include <moveit/ompl_interface/ompl_interface.h>
// #include <moveit/ompl_interface/model_based_planning_context.h>

#define ROBOWFLEX_AT_LEAST_INDIGO ROS_VERSION_MINIMUM(1, 11, 0)
#define ROBOWFLEX_AT_LEAST_LUNAR ROS_VERSION_MINIMUM(1, 12, 0)
#define ROBOWFLEX_AT_LEAST_KINETIC ROS_VERSION_MINIMUM(1, 13, 0)
#define ROBOWFLEX_AT_LEAST_MELODIC ROS_VERSION_MINIMUM(1, 14, 0)

namespace robowflex
{
    class Exception : public std::exception
    {
    public:
        Exception(int value, const std::string &message) : value_(value), message_(message)
        {
        }

        int getValue() const
        {
            return value_;
        }

        const std::string &getMessage() const
        {
            return message_;
        }

        virtual const char *what() const throw()
        {
            return message_.c_str();
        }

    protected:
        const int value_;
        const std::string message_;
    };

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

        // Creates a file
        std::ofstream createFile(const std::string &file);

        template <typename T>
        bool messageToYAMLFile(T &msg, const std::string &file)
        {
            YAML::Node yaml;
            yaml = msg;

            YAML::Emitter out;
            out << yaml;

            std::ofstream fout(file);
            fout << out.c_str();
            fout.close();

            return true;
        }

        template <typename T>
        bool YAMLFileToMessage(T &msg, const std::string &file)
        {
            const auto &result = IO::loadFileToYAML(file);

            if (!result.first)
                return false;

            msg = result.second.as<T>();
            return true;
        }

        class Handler
        {
        public:
            Handler(const std::string &name);

            Handler(Handler const &) = delete;
            void operator=(Handler const &) = delete;

            ~Handler();

            // Loads an YAML node to the ROS parameter server.
            void loadYAMLtoROS(const YAML::Node &node, const std::string &prefix = "");

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

        Geometry(ShapeType::Type type, const Eigen::Vector3d &dimensions, const std::string &resource = "");

        Geometry(const Geometry &) = delete;             // non construction-copyable
        Geometry &operator=(const Geometry &) = delete;  // non copyable

        const bool isMesh() const;

        const shape_msgs::SolidPrimitive getSolidMsg() const;
        const shape_msgs::Mesh getMeshMsg() const;

        const shapes::ShapePtr &getShape() const
        {
            return shape_;
        }

    private:
        shapes::Shape *loadShape() const;

        ShapeType::Type type_{ShapeType::Type::BOX};                 // Geometry Type.
        std::string resource_{""};                                   // Resource locator for MESH types.
        const Eigen::Vector3d dimensions_{Eigen::Vector3d::Ones()};  // Dimensions to scale geometry along axes.
        const shapes::ShapePtr shape_{nullptr};                      // Loaded shape.
    };

    namespace TF
    {
        Eigen::Vector3d vectorMsgToEigen(const geometry_msgs::Vector3 &msg);
        geometry_msgs::Vector3 vectorEigenToMsg(const Eigen::Vector3d &vector);
        Eigen::Affine3d poseMsgToEigen(const geometry_msgs::Pose &msg);
        geometry_msgs::Pose poseEigenToMsg(const Eigen::Affine3d &pose);
        Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion &msg);
        geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond &quaternion);

        moveit_msgs::BoundingVolume getBoundingVolume(const Eigen::Affine3d &poses, const Geometry &geometries);
        moveit_msgs::PositionConstraint getPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                                              const Eigen::Affine3d &poses, const Geometry &geometries);
        moveit_msgs::OrientationConstraint getOrientationConstraint(const std::string &ee_name,
                                                                    const std::string &base_name,
                                                                    const Eigen::Quaterniond &orientation,
                                                                    const Eigen::Vector3d &tolerances);
    }  // namespace TF

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

    private:
        planning_scene::PlanningScenePtr scene_;
    };

    class Planner
    {
    public:
        Planner(Robot &robot) : robot_(robot), handler_(robot_.getHandler())
        {
        }

        Planner(Planner const &) = delete;
        void operator=(Planner const &) = delete;

        virtual planning_interface::MotionPlanResponse plan(const Scene &scene,
                                                            const planning_interface::MotionPlanRequest &request) = 0;

        virtual const std::vector<std::string> getPlannerConfigs() const = 0;

        const Robot &getRobot() const
        {
            return robot_;
        }

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

        planning_interface::MotionPlanResponse plan(const Scene &scene,
                                                    const planning_interface::MotionPlanRequest &request) override;

    protected:
        planning_pipeline::PlanningPipelinePtr pipeline_;
    };

    namespace OMPL
    {
        class Settings
        {
        public:
            // Initialized here so default arguments are parsed correctly in loadPipeline.
            Settings()
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

        class OMPLPipelinePlanner : public PipelinePlanner
        {
        public:
            OMPLPipelinePlanner(Robot &robot);

            OMPLPipelinePlanner(OMPLPipelinePlanner const &) = delete;
            void operator=(OMPLPipelinePlanner const &) = delete;

            bool initialize(const std::string &config_file = "", const Settings settings = Settings(),
                            const std::string &plugin = "ompl_interface/OMPLPlanner",
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

            const std::vector<std::string> getPlannerConfigs() const override;

        protected:
            static const std::vector<std::string> DEFAULT_ADAPTERS;

        private:
            std::vector<std::string> configs_;
        };

        // class OMPLInterfacePlanner : public Planner
        // {
        // public:
        //     OMPLInterfacePlanner(Robot &robot);

        //     OMPLInterfacePlanner(OMPLInterfacePlanner const &) = delete;
        //     void operator=(OMPLInterfacePlanner const &) = delete;

        //     bool initialize(const std::string &config_file = "", const OMPL::Settings settings = Settings());

        //     planning_interface::MotionPlanResponse plan(Scene &scene,
        //                                                 const planning_interface::MotionPlanRequest &request)
        //                                                 override;

        //     const std::vector<std::string> getPlannerConfigs() const override;

        // private:
        //     ompl_interface::OMPLInterface interface_;
        //     std::vector<std::string> configs_;
        // };
    }  // namespace OMPL

    class MotionRequestBuilder
    {
    public:
        MotionRequestBuilder(const Planner &planner, const std::string &group_name);

        void setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &wp);
        void setStartConfiguration(const std::vector<double> &joints);
        void setGoalConfiguration(const std::vector<double> &joints);
        void setGoalRegion(const std::string &ee_name, const std::string &base_name, const Eigen::Affine3d &pose,
                           const Geometry &geometry, const Eigen::Quaterniond &orientation,
                           const Eigen::Vector3d &tolerances);
        void addPathPoseConstraint(const std::string &ee_name, const std::string &base_name,
                                   const Eigen::Affine3d &pose, const Geometry &geometry,
                                   const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);
        void addPathPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                       const Eigen::Affine3d &pose, const Geometry &geometry);
        void addPathOrientationConstraint(const std::string &ee_name, const std::string &base_name,
                                          const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);
        const planning_interface::MotionPlanRequest &getRequest() const;

        bool toYAMLFile(const std::string &file);
        bool fromYAMLFile(const std::string &file);

    private:
        const Planner &planner_;
        const Robot &robot_;
        const std::string group_name_;
        const robot_model::JointModelGroup *jmg_;

        planning_interface::MotionPlanRequest request_;

        static const std::string DEFAULT_CONFIG;
    };

    // Forward Declaration.
    class BenchmarkOutputter;

    class Benchmarker
    {
    public:
        class Options
        {
        public:
            Options() : runs(100)
            {
            }

            unsigned int runs;
        };

        class Results
        {
        public:
            class Run
            {
            public:
                Run(const std::string &name, double time, bool success, bool correct, double length, double clearance,
                    double smoothness)
                  : name(name)
                  , time(time)
                  , success(success)
                  , correct(correct)
                  , length(length)
                  , clearance(clearance)
                  , smoothness(smoothness)
                {
                }

                const std::string name;
                const double time;
                /** Whether or not MoveIt returns a 'success'. */
                const bool success;
                /** True if the path is actually collision free. */
                const bool correct;
                const double length;
                const double clearance;
                const double smoothness;
            };

            Results()
            {
            }

            void addRun(const Scene &scene, const std::string &name, double time,
                        planning_interface::MotionPlanResponse &run);
            void computeMetric(const Scene &scene, planning_interface::MotionPlanResponse &run, bool &correct,
                               double &length, double &clearance, double &smoothness);

            std::vector<Run> runs;
        };

        Benchmarker();

        void addBenchmarkingRequest(const std::string &name, Scene &scene, Planner &planner,
                                    MotionRequestBuilder &request);

        void benchmark(BenchmarkOutputter &output, const Options &options = Options());

    private:
        std::map<std::string, std::tuple<Scene &, Planner &, MotionRequestBuilder &>> requests_;
    };

    class BenchmarkOutputter
    {
    public:
        BenchmarkOutputter(const std::string &output_path) : output_path_(output_path)
        {
        }

        // Write one unit of output (usually a single planner) to the output.
        virtual void writeOutput(const Benchmarker::Results &results, const std::string &name, const Scene &scene, const Planner &planner, const MotionRequestBuilder &builder) = 0;

        // Close resources the outputter is using.
        virtual void close() = 0;

    protected:
        const std::string output_path_;
    };

    class JSONBenchmarkOutputter : public BenchmarkOutputter
    {
    public:
        JSONBenchmarkOutputter(const std::string &output_path) : BenchmarkOutputter(output_path)
        {
        }

        void writeOutput(const Benchmarker::Results &results, const std::string &name, const Scene &scene, const Planner &planner, const MotionRequestBuilder &builder) override; 

        void close() override;

    private:
        bool is_init = false;

        std::ofstream outfile_;
    };

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

}  // namespace robowflex

#include "yaml.h"

#endif
