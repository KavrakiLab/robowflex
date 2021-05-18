/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ROBOT_
#define ROBOWFLEX_ROBOT_

#include <string>  // for std::string
#include <vector>  // for std::vector
#include <map>     // for std::map

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf/model.h>
#include <srdfdom/model.h>
#include <tinyxml2.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>
#include <robowflex_library/constants.h>
#include <robowflex_library/io/handler.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /** \endcond */

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    /** \endcond */

    /** \class robowflex::RobotPtr
        \brief A shared pointer wrapper for robowflex::Robot. */

    /** \class robowflex::RobotConstPtr
        \brief A const shared pointer wrapper for robowflex::Robot. */

    /** \brief Loads information about a robot and maintains information about a robot's state.
     *
     *  The Robot class is a wrapper around _MoveIt!_'s robot_model::RobotModel and a "scratch"
     * robot_state::RobotState. Mostly, this class handles loading the relevant information to the parameter
     * server without the use of a launch file. The necessary files are the URDF and SRDF (both either XML or
     * xacro files), and the joint limits and kinematics plugin (both YAML files). These pieces of information
     * are loaded to the parameter server under the name provided to the constructor so that multiple robots
     * can be used. Note that all information is also loaded under the unique namespace generated by the
     * IO::Handler.
     *
     *  By default, no IK kinematics plugins are loaded due to startup costs (particularly for R2). You must
     * call loadKinematics() with the desired planning groups (as defined by the SRDF) if you want IK.
     *
     *  Additionally, post-processing function hooks are provided if modifications need to be done to any of
     * the necessary files. This enables the addition of extra joints, semantic information, joint limit
     * overloading, etc. as needed. These functions are called after the robot is initially loaded, so it is
     * possible to access robot model information in these functions. For example, see
     * setSRDFPostProcessAddPlanarJoint().
     */
    class Robot
    {
    public:
        /** \brief A function that runs after loading a YAML file and can modify its contents. Returns true on
         * success, false on failure.
         */
        typedef std::function<bool(YAML::Node &)> PostProcessYAMLFunction;

        /** \brief A function that runs after loading a XML file and can modify its contents. Returns true on
         * success, false on failure.
         */
        typedef std::function<bool(tinyxml2::XMLDocument &)> PostProcessXMLFunction;

        static const std::string ROBOT_DESCRIPTION;  ///< Default robot description name.
        static const std::string ROBOT_SEMANTIC;     ///< Default robot semantic description suffix.
        static const std::string ROBOT_PLANNING;     ///< Default robot planning description suffix.
        static const std::string ROBOT_KINEMATICS;   ///< Default robot kinematics description suffix.

        /** \brief Constructor.
         *  \param[in] name The name of the robot. Used to namespace information under.
         */
        Robot(const std::string &name);

        // non-copyable
        Robot(Robot const &) = delete;
        void operator=(Robot const &) = delete;

        /** \name Initialization and Loading
            \{ */

        /** \brief Initializes a robot from a kinematic description.
         *  A default semantic description is used.
         *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
         *  \return True on success, false on failure.
         */
        bool initialize(const std::string &urdf_file);

        /** \brief Initializes a robot from a kinematic and semantic description.
         *  All files are loaded under the robot's namespace.
         *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
         *  \param[in] srdf_file Location of the robot's SRDF (XML or .xacro file).
         *  \return True on success, false on failure.
         */
        bool initialize(const std::string &urdf_file, const std::string &srdf_file);

        /** \brief Initialize a robot with a kinematics description.
         *  \param[in] kinematics_file Location of the kinematics plugin information (a YAML file).
         *  \return True on success, false on failure.
         */
        bool initializeKinematics(const std::string &kinematics_file);

        /** \brief Initializes a robot from a kinematic and semantic description.
         *  All files are loaded under the robot's namespace.
         *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
         *  \param[in] srdf_file Location of the robot's SRDF (XML or .xacro file).
         *  \param[in] limits_file Location of the joint limit information (a YAML file). Optional.
         *  \param[in] kinematics_file Location of the kinematics plugin information (a YAML file). Optional.
         *  \return True on success, false on failure.
         */
        bool initialize(const std::string &urdf_file, const std::string &srdf_file,
                        const std::string &limits_file = "", const std::string &kinematics_file = "");

        /** \brief Initializes a robot from a YAML config which includes URDF (urdf) and optional the SRDF
         * (srdf), joint limits (joint_limits), IK plugins (kinematics) and a default state (robot_state).
         * All files are loaded under the robot's namespace. The names of the YAML keys are in parenthesis.
         * \param[in] config_file Location of the yaml config file.
         * \return True on success, false on failure.
         */
        bool initializeFromYAML(const std::string &config_file);

        /** \brief Loads a YAML file into the robot's namespace under \a name.
         *  \param[in] name Name to load file under.
         *  \param[in] file File to load.
         *  \return True on success, false on failure.
         */
        bool loadYAMLFile(const std::string &name, const std::string &file);

        /** \brief Loads a YAML file into the robot's namespace under \a name, with a post-process function.
         *  \param[in] name Name to load file under.
         *  \param[in] file File to load.
         *  \param[in] function Optional post processing function.
         *  \return True on success, false on failure.
         */
        bool loadYAMLFile(const std::string &name, const std::string &file,
                          const PostProcessYAMLFunction &function);

        /** \brief Loads an XML or .xacro file into a string.
         *  \param[in] file File to load.
         *  \return XML string upon success, empty string on failure.
         */
        std::string loadXMLFile(const std::string &file);

        /** \brief Sets a post processing function for loading the URDF.
         *  \param[in] function The function to use.
         */
        void setURDFPostProcessFunction(const PostProcessXMLFunction &function);

        /** \brief Checks if a node link exist with named name_link
         *  \param[in] doc The URDF description.
         *  \param[in] name The name of the link to find.
         *  \return True if link exists, false otherwise.
         */
        bool isLinkURDF(tinyxml2::XMLDocument &doc, const std::string &name);

        /** \brief Sets a post processing function for loading the SRDF.
         *  \param[in] function The function to use.
         */
        void setSRDFPostProcessFunction(const PostProcessXMLFunction &function);

        /** \brief Sets a post processing function for loading the joint limits file.
         *  \param[in] function The function to use.
         */
        void setLimitsPostProcessFunction(const PostProcessYAMLFunction &function);

        /** \brief Sets a post processing function for loading the kinematics plugin file.
         *  \param[in] function The function to use.
         */
        void setKinematicsPostProcessFunction(const PostProcessYAMLFunction &function);

        /** \brief Adds a planar virtual joint through the SRDF to the loaded robot with name \a name. This
         * joint will have three degrees of freedom: <name>/x, <name>/y, and <name>/theta. Will apply this
         * joint between the world and the root frame.
         *  \param[in] name Name for new joint.
         */
        void setSRDFPostProcessAddPlanarJoint(const std::string &name);

        /** \brief Adds a planar virtual joint through the SRDF to the loaded robot with name \a name. This
         * joint will have three degrees of freedom: <name>/x, <name>/y, and <name>/theta. Will apply this
         * joint between the world and the root frame.
         *  \param[in] name Name for new joint.
         */
        void setSRDFPostProcessAddFloatingJoint(const std::string &name);

        /** \brief Loads the kinematics plugin for a joint group. No kinematics are loaded by default.
         *  \param[in] group Joint group name to load.
         *  \return True on success, false on failure.
         */
        bool loadKinematics(const std::string &group);

        /** \} */

        /** \name Getters and Setters
            \{ */

        /** \brief Get the robot's model name.
         *  \return The robot's model name.
         */
        const std::string &getModelName() const;

        /** \brief Get the robot's name.
         *  \return The robot's name.
         */
        const std::string &getName() const;

        /** \brief Get a const reference to the loaded robot model.
         *  \return The robot model.
         */
        const robot_model::RobotModelPtr &getModelConst() const;

        /** \brief Get a reference to the loaded robot model.
         *  \return The robot model.
         */
        robot_model::RobotModelPtr &getModel();

        /** \brief Get the raw URDF Model.
         *  \return The URDF Model.
         */
        urdf::ModelInterfaceConstSharedPtr getURDF() const;

        /** \brief Get the raw URDF Model as a string.
         *  \return The URDF Model as a string.
         */
        const std::string &getURDFString() const;

        /** \brief Get the raw SRDF Model.
         *  \return The SRDF model.
         */
        srdf::ModelConstSharedPtr getSRDF() const;

        /** \brief Get the raw SRDF Model as a string.
         *  \return The SRDF model as a string.
         */
        const std::string &getSRDFString() const;

        /** \brief Get a const reference to the scratch robot state.
         *  \return The scratch robot state.
         */
        const robot_model::RobotStatePtr &getScratchStateConst() const;

        /** \brief Get a reference to the scratch robot state.
         *  \return The scratch robot state.
         */
        robot_model::RobotStatePtr &getScratchState();

        /** \brief Allocate a new robot state.
         *  \return The new robot state.
         */
        robot_model::RobotStatePtr allocState() const;

        /** \brief Get the underlying IO handler used for this robot.
         *  \return A reference to the IO handler.
         */
        const IO::Handler &getHandlerConst() const;

        /** \brief Get the underlying IO handler used for this robot.
         *  \return A reference to the IO handler.
         */
        IO::Handler &getHandler();

        /** \} */

        /** \name Robot State Operations
            \{ */

        /** \brief Sets the scratch state from a vector of joint positions (all must be specified)
         *  \param[in] positions Joint positions to set.
         */
        void setState(const std::vector<double> &positions);

        /** \brief Sets the scratch state from a map of joint name to position.
         *  \param[in] variable_map Joint positions to set.
         */
        void setState(const std::map<std::string, double> &variable_map);

        /** \brief Sets the scratch state from a vector of joint names and their positions.
         *  \param[in] variable_names Joint names.
         *  \param[in] variable_position Position of joint variable (index matches entry in \a variable_names)
         */
        void setState(const std::vector<std::string> &variable_names,
                      const std::vector<double> &variable_position);

        /** \brief Sets the scratch state from a robot state message.
         *  \param[in] state The state to set.
         */
        void setState(const moveit_msgs::RobotState &state);

        /** \brief Sets the scratch state from a robot state message saved to a YAML file.
         *  \param[in] file The YAML file to load.
         */
        void setStateFromYAMLFile(const std::string &file);

        /** \brief Sets the group of the scratch state to a vector of joint positions.
         *  \param[in] name Name of group to set.
         *  \param[in] positions Positions to set.
         */
        void setGroupState(const std::string &name, const std::vector<double> &positions);

        /** \brief Gets the current joint positions of the scratch state.
         *  \return A vector of joint positions.
         */
        std::vector<double> getState() const;

        /** \brief Gets the names of joints of the robot.
         *  \return A vector of joint names.
         */
        std::vector<std::string> getJointNames() const;

        /** \brief Checks if a joint exists in the robot.
         *  \return True if the joint exists, false otherwise.
         */
        bool hasJoint(const std::string &joint) const;

        /** \brief Get the current pose of a link on the scratch state.
         *  \param[in] name The name of the link to find the transform of.
         *  \return The transform of link \a name.
         */
        const RobotPose &getLinkTF(const std::string &name) const;

        /** \brief Get the current pose of a link \a target in the frame of \a base.
         *  \param[in] base The link to use as the base frame.
         *  \param[in] target The link to find the transform of.
         *  \return The transform of link \a target in the frame of \a base.
         */
        const RobotPose getRelativeLinkTF(const std::string &base, const std::string &target) const;

        /** \} */

        /** \name Inverse Kinematics
            \{ */

        /** \brief Robot IK Query options.
         *  IK queries in Robowflex consist of:
         *   a) A position specified by some geometric region (a robowflex::Geometry) at a pose.
         *   b) An orientation specified by some base orientation with allowable deviations specified by
         *      tolerances on the XYZ Euler axes.
         *  It is recommended to use the provided constructors to specify a query, or to use the addRequest()
         *  function. Multiple target tips can be specified, but note that not all kinematics solvers support
         *  multi-tip IK. Additionally, a robowflex::Scene can be specified to do collision-aware IK.
         */
        struct IKQuery
        {
            /** \name Query Targets
                \{ */

            std::string group;                             ///< Target joint group to do IK for.
            std::vector<std::string> tips;                 ///< List of end-effectors.
            std::vector<GeometryConstPtr> regions;         ///< Regions to sample target positions from.
            RobotPoseVector region_poses;                  ///< Poses of regions.
            std::vector<Eigen::Quaterniond> orientations;  ///< Target orientations.
            EigenSTL::vector_Vector3d tolerances;          ///< XYZ Euler orientation tolerances.

            /** \} */

            /** \name Additional Solver Options
                \{ */

            ScenePtr scene;                     ///< If provided, use this scene for collision checking.
            bool verbose{false};                ///< Verbose output of collision checking.
            bool random_restart{true};          ///< Randomly reset joint states.
            bool approximate_solutions{false};  ///< Return approximate solutions.
            std::size_t attempts{constants::ik_attempts};  ///< IK attempts (samples within regions).
            double timeout{0.};                            ///< Timeout for each query.

            /** \} */

            /** Constructor. Initialize a basic IK query to reach the desired \a pose.
             *  \param[in] group Group to set.
             *  \param[in] pose Desired pose of end-effector.
             *  \param[in] radius Radius tolerance around position.
             *  \param[in] tolerances Tolerance about \a orientation.
             */
            IKQuery(const std::string &group, const RobotPose &pose, double radius = constants::ik_tolerance,
                    const Eigen::Vector3d &tolerance = constants::ik_rot_tolerance);

            /** Constructor. Initialize a basic IK query to reach the desired \a position and \a orientation.
             *  \param[in] group Group to set.
             *  \param[in] position Position to achieve.
             *  \param[in] orientation Mean orientation.
             *  \param[in] radius Radius tolerance around position.
             *  \param[in] tolerances Tolerance about \a orientation.
             */
            IKQuery(const std::string &group, const Eigen::Vector3d &position,
                    const Eigen::Quaterniond &orientation, double radius = constants::ik_tolerance,
                    const Eigen::Vector3d &tolerance = constants::ik_rot_tolerance);

            /** Constructor. Initialize an  IK query to reach somewhere in the provided \a region (at a \a
             *  pose) and \a orientation.
             *  \param[in] group Group to set.
             *  \param[in] region Region of points for position.
             *  \param[in] pose Pose of the \a region.
             *  \param[in] orientation Mean orientation.
             *  \param[in] tolerances Tolerance about \a orientation.
             *  \return True on success, false on failure.
             */
            IKQuery(const std::string &group, const GeometryConstPtr &region, const RobotPose &pose,
                    const Eigen::Quaterniond &orientation,
                    const Eigen::Vector3d &tolerance = constants::ik_rot_tolerance,
                    const ScenePtr &scene = nullptr, bool verbose = false);

            /** Constructor. Initialize a basic multi-target IK query so that each of the \a tips reach their
             *  desired \a poses.
             *  \param[in] group Group to set.
             *  \param[in] poses Desired poses of end-effector tips.
             *  \param[in] input_tips End-effector tips to target.
             *  \param[in] radius Radius tolerance around position.
             *  \param[in] tolerances Tolerance about \a orientation.
             *  \return True on success, false on failure.
             */
            IKQuery(const std::string &group, const RobotPoseVector &poses,
                    const std::vector<std::string> &input_tips, double radius = constants::ik_tolerance,
                    const Eigen::Vector3d &tolerance = constants::ik_rot_tolerance,
                    const ScenePtr &scene = nullptr, bool verbose = false);

            /** \brief Add a request for a \a tip.
             *  \param[in] tip Tip for the request.
             *  \param[in] region Region of points for position.
             *  \param[in] pose Pose of the \a region.
             *  \param[in] orientation Mean orientation.
             *  \param[in] tolerances Tolerance about \a orientation.
             */
            void addRequest(const std::string &tip, const GeometryConstPtr &region, const RobotPose &pose,
                            const Eigen::Quaterniond &orientation,
                            const Eigen::Vector3d &tolerance = constants::ik_rot_tolerance);

            /** \brief Sample a desired end-effector pose for the \a index -th region.
             *  \param[out] pose The sampled pose.
             *  \param[in] index The index of the region. Should be less than numTargets().
             *  \return True on success, false on failure.
             */
            bool sampleInRegion(RobotPose &pose, std::size_t index = 0) const;

            /** \brief Get the number of IK targets in this query.
             *  \return The number of IK targets.
             */
            std::size_t numTargets() const;

            /** \brief Get the group state validity callback function used by collision-aware IK.
             *  \return If scene is not null, the GSVCF. Otherwise, an empty function.
             */
            moveit::core::GroupStateValidityCallbackFn getGSVCF() const;
        };

        /** \brief Sets a group of the scratch state from an IK query. If the IK query fails the scratch state
         *  retains its initial value.
         *  \param[in] query Query for inverse kinematics. See Robot::IKQuery documentation for more.
         *  \return True on success, false on failure.
         */
        bool setFromIK(const IKQuery &query);

        /** \brief Sets a group a robot state from an IK query. If the IK query fails the scratch state
         *  retains its initial value.
         *  \param[in] query Query for inverse kinematics. See Robot::IKQuery documentation for more.
         *  \param[out] state Robot state to set from IK.
         *  \return True on success, false on failure.
         */
        bool setFromIK(const IKQuery &query, robot_state::RobotState &state);

        /** \} */

        /** \name IO
            \{ */

        /** \brief Dumps the current configuration of the robot as a YAML file.
         *  \param[in] file File to write to.
         *  \return True on success, false on failure.
         */
        bool toYAMLFile(const std::string &file) const;

        /** \brief Dumps the names of links and absolute paths to their visual mesh files to a YAML file.
         *  \param[in] file File to save to.The name of the link to find the transform of.
         *  \return True on success, false on failure.
         */
        bool dumpGeometry(const std::string &file) const;

        /** \brief Dumps the tranforms of all links of a robot at its current state to a file.
         *  \param[in] filename Filename to output to.
         *  \return True on success, false on failure.
         */
        bool dumpTransforms(const std::string &filename) const;

        /** \brief Dumps the tranforms of all links of a robot through a robot trajectory to a file.
         *  \param[in] path Path to output.
         *  \param[in] filename Filename to output to.
         *  \param[in] fps The transforms (frames) per second used to interpolate the given path.
         *  \param[in] threshold The minimum distance between states before transforms are output.
         *  \return True on success, false on failure.
         */
        bool dumpPathTransforms(const robot_trajectory::RobotTrajectory &path, const std::string &filename,
                                double fps = 30, double threshold = 0.0) const;

        /** \brief Dumps the current scratch configuration of the robot to a YAML file compatible with a
         * scene.
         *  \param[in] filename Filename to output to.
         *  \return True on success, false on failure.
         */
        bool dumpToScene(const std::string &filename) const;

        /** \} */

    protected:
        /** \name Protected Initialization
            \{ */

        /** \brief Loads the URDF file.
         *  \param[in] urdf_file The URDF file name.
         *  \return True on success, false on failure.
         */
        bool loadURDFFile(const std::string &urdf_file);

        /** \brief Loads the SRDF file.
         *  \param[in] srdf_file The SRDF file name.
         *  \return True on success, false on failure.
         */
        bool loadSRDFFile(const std::string &srdf_file);

        /** \brief Initializes and loads the robot. Calls post-processing functions and creates scratch state.
         *  \param[in] namespaced Whether or not the parameter server description is under the handler
         * namespace.
         */
        void initializeInternal(bool namespaced = true);

        /** \brief Loads a robot model from the loaded information on the parameter server.
         *  \param[in] description Robot description on parameter server.
         */
        void loadRobotModel(const std::string &description);

        /** \brief Updates a loaded XML string based on an XML post-process function. Called after initial,
         * unmodified robot is loaded.
         *  \param[in,out] string Input XML string.
         *  \param[in] function XML processing function.
         */
        void updateXMLString(std::string &string, const PostProcessXMLFunction &function);

        /** \} */

        const std::string name_;  ///< Robot name.
        IO::Handler handler_;     ///< IO handler (namespaced with \a name_)

        std::string urdf_;  ///< The URDF as a string.
        std::string srdf_;  ///< The SRDF as a string.

        PostProcessXMLFunction urdf_function_;         ///< URDF post-processing function.
        PostProcessXMLFunction srdf_function_;         ///< SRDF post-processing function.
        PostProcessYAMLFunction limits_function_;      ///< Limits YAML post-processing function.
        PostProcessYAMLFunction kinematics_function_;  ///< Kinematics plugin YAML post-processing function.

        std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;    ///< Robot model loader.
        robot_model::RobotModelPtr model_;                                ///< Loaded robot model.
        std::map<std::string, robot_model::SolverAllocatorFn> imap_;      ///< Kinematic solver allocator map.
        kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_;  ///< Kinematic plugin loader.

        robot_state::RobotStatePtr scratch_;  ///< Scratch robot state.
    };

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(ParamRobot);
    /** \endcond */

    /** \class robowflex::ParamRobotPtr
        \brief A shared pointer wrapper for robowflex::ParamRobot. */

    /** \class robowflex::ParamRobotConstPtr
        \brief A const shared pointer wrapper for robowflex::ParamRobot. */

    /** \brief Loads information about a robot from the parameter server.
     */
    class ParamRobot : public Robot
    {
    public:
        /** Constructor. Loads robot from parameter server.
         *  \param[in] name Name for this robot.
         */
        ParamRobot(const std::string &name = "DEFAULT");
    };
}  // namespace robowflex

#endif
