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
     */
    class Robot
    {
    public:
        typedef std::function<bool(YAML::Node &)> PostProcessYAMLFunction;
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

        /** \brief Initializes a robot from a kinematic and semantic description.
         *  All files are loaded under the robot's namespace.
         *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
         *  \param[in] srdf_file Location of the robot's SRDF (XML or .xacro file).
         *  \param[in] limits_file Location of the joint limit information (a YAML file).
         *  \param[in] kinematics_file Location of the kinematics plugin information (a YAML file).
         *  \return True on success, false on failure.
         */
        bool initialize(const std::string &urdf_file, const std::string &srdf_file,
                        const std::string &limits_file, const std::string &kinematics_file);

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

        /** \brief Loads an XML or .xacro file into the robot's namespace under \a name, with a post-process
         *  function.
         *  \param[in] name Name to load file under.
         *  \param[in] file File to load.
         *  \return True on success, false on failure.
         */
        bool loadXMLFile(const std::string &name, const std::string &file);

        /** \brief Loads an XML or .xacro file into the robot's namespace under \a name.
         * \param[in] name Name to load file under.
         * \param[in] file File to load.
         * \param[in] function Optional post processing function.
         * \return True on success, false on failure.
         */
        bool loadXMLFile(const std::string &name, const std::string &file,
                         const PostProcessXMLFunction &function);

        /** \brief Sets a post processing function for loading the URDF.
         *  \param[in] function The function to use.
         */
        void setURDFPostProcessFunction(const PostProcessXMLFunction &function);

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

        /** \brief Get the raw SRDF Model.
         *  \return The SRDF model.
         */
        srdf::ModelConstSharedPtr getSRDF() const;

        /** \brief Get a const reference to the scratch robot state.
         *  \return The scratch robot state.
         */
        const robot_model::RobotStatePtr &getScratchState() const;

        /** \brief Get a reference to the scratch robot state.
         *  \return The scratch robot state.
         */
        robot_model::RobotStatePtr &getScratchState();

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

        /** \brief Sets the group of the scratch state to a vector of joint positions.
         *  \param[in] name Name of group to set.
         *  \param[in] positions Positions to set.
         */
        void setGroupState(const std::string &name, const std::vector<double> &positions);

        /** \brief Sets a group of the scratch state from an IK query.
         *  Position of query is specified by a geometry \a region at a \a pose, and orientation is set by \a
         *  orientation with XYZ Euler angle tolerances from \a tolerances.
         *  \param[in] group Group to set.
         *  \param[in] region Region of points for position.
         *  \param[in] pose Pose of the \a region.
         *  \param[in] orientation Mean orientation
         *  \param[in] tolerances Tolerance about \a orientation.
         *  \return True on success, false on failure.
         */
        bool setFromIK(const std::string &group, const GeometryConstPtr &region, const Eigen::Affine3d &pose,
                       const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances);

        /** \brief Gets the current joint positions of the scratch state.
         *  \return A vector of joint positions.
         */
        std::vector<double> getState() const;

        /** \brief Gets the names of joints of the robot.
         *  \return A vector of joint names.
         */
        std::vector<std::string> getJointNames() const;

        /** \brief Get the current pose of a link on the scratch state.
         *  \param[in] name The name of the link to find the transform of.
         *  \return The transform of link \a name.
         */
        const Eigen::Affine3d &getLinkTF(const std::string &name) const;

        /** \brief Get the current pose of a link \a target in the frame of \a base.
         *  \param[in] base The link to use as the base frame.
         *  \param[in] target The link to find the transform of.
         *  \return The transform of link \a target in the frame of \a base.
         */
        const Eigen::Affine3d getRelativeLinkTF(const std::string &base, const std::string &target) const;

        /** \brief Checks if the scratch state is in collision in \a scene.
         *  \param[in] scene Scene to check collision against.
         *  \return True if in collision, false otherwise.
         */
        bool inCollision(const SceneConstPtr &scene) const;

        /** \} */

        /** \name IO
            \{ */

        /** \brief Dumps the names of links and absolute paths to their visual mesh files to a YAML file.
         *  \param[in] file File to save to.The name of the link to find the transform of.
         *  \return True on success, false on failure.
         */
        bool dumpGeometry(const std::string &file) const;

        /** \brief Dumps a the tranforms of all links of a robot through a robot trajectory to a file.
         *  \param[in] path Path to output.
         *  \param[in] filename Filename to output to.
         *  \param[in] fps The transforms (frames) per second used to interpolate the given path.
         *  \param[in] threshold The minimum distance between states before transforms are output.
         *  \return True on success, false on failure.
         */
        bool dumpPathTransforms(const robot_trajectory::RobotTrajectory &path, const std::string &filename,
                                double fps = 30, double threshold = 0.0);

        /** \} */

    protected:
        /** \brief Loads robot description files to parameter server.
         *  All files are loaded under the robot's namespace.
         *  \param[in] urdf_file Location of the robot's URDF (XML or .xacro file).
         *  \param[in] srdf_file Location of the robot's SRDF (XML or .xacro file).
         *  \param[in] limits_file Location of the joint limit information (a YAML file).
         *  \param[in] kinematics_file Location of the kinematics plugin information (a YAML file).
         *  \return True on success, false on failure.
         */
        bool loadRobotDescription(const std::string &urdf_file, const std::string &srdf_file,
                                  const std::string &limits_file, const std::string &kinematics_file);

        /** \brief Loads a robot model from the loaded information on the parameter server.
         */
        void loadRobotModel(bool namespaced = true);

        const std::string name_;  ///< Robot name.
        IO::Handler handler_;     ///< IO handler (namespaced with \a name_)

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
