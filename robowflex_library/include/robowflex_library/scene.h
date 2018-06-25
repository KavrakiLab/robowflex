/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_SCENE_
#define ROBOWFLEX_SCENE_

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    /** \endcond */

    /** \class robowflex::RobotPtr
        \brief A shared pointer wrapper for robowflex::Robot. */

    /** \class robowflex::RobotConstPtr
        \brief A const shared pointer wrapper for robowflex::Robot. */

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    /** \class robowflex::ScenePtr
        \brief A shared pointer wrapper for robowflex::Scene. */

    /** \class robowflex::SceneConstPtr
        \brief A const shared pointer wrapper for robowflex::Scene. */

    /** \brief Loads information about a robot and maintains information about a robot's state.
     */
    class Robot
    {
    public:
        /** \brief Constructor.
         *  \param[in] name The name of the robot. Used to namespace information under.
         */
        Robot(const std::string &name);

        // non-copyable
        Robot(Robot const &) = delete;
        void operator=(Robot const &) = delete;

        /** \name Initialization and loading.
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
         * \param[in] name Name to load file under.
         * \param[in] file File to load.
         * \return True on success, false on failure.
         */
        bool loadYAMLFile(const std::string &name, const std::string &file);

        /** \brief Loads an XML or .xacro file into the robot's namespace under \a name.
         * \param[in] name Name to load file under.
         * \param[in] file File to load.
         * \return True on success, false on failure.
         */
        bool loadXMLFile(const std::string &name, const std::string &file);

        /** \brief Loads the kinematics plugin for a joint group. No kinematics are loaded by default.
         *  \param[in] group Joint group name to load.
         *  \return True on success, false on failure.
         */
        bool loadKinematics(const std::string &group);

        /** \} */

        /** \name Getters and Setters
            \{ */

        /** \brief Get the robot's name.
         *  \return The robot's name.
         */
        const std::string &getName() const
        {
            return name_;
        }

        /** \brief Get a const reference to the loaded robot model.
         *  \return The robot model.
         */
        const robot_model::RobotModelPtr &getModelConst() const
        {
            return model_;
        }

        /** \brief Get a reference to the loaded robot model.
         *  \return The robot model.
         */
        robot_model::RobotModelPtr &getModel()
        {
            return model_;
        }

        /** \brief Get a const reference to the scratch robot state.
         *  \return The scratch robot state.
         */
        const robot_model::RobotStatePtr &getScratchState() const
        {
            return scratch_;
        }

        /** \brief Get a reference to the scratch robot state.
         *  \return The scratch robot state.
         */
        robot_model::RobotStatePtr &getScratchState()
        {
            return scratch_;
        }

        /** \brief Get the underlying IO handler used for this robot.
         *  \return A reference to the IO handler.
         */
        IO::Handler &getHandler()
        {
            return handler_;
        }

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
         */
        void setFromIK(const std::string &group, const Geometry &region, const Eigen::Affine3d &pose,
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
         *  \return The transform of link \a name.
         */
        const Eigen::Affine3d &getLinkTF(const std::string &name) const;

        /** \brief Checks if the scratch state is in collision in \a scene.
         *  \param[in] scene Scene to check collision against.
         *  \return True if in collision, false otherwise.
         */
        bool inCollision(const SceneConstPtr &scene) const;

        /** \} */

        /** \name IO
            \{ */

        /** \brief Dumps the names of links and absolute paths to their visual mesh files to a YAML file.
         *  \param[in] file File to save to.
         *  \return True on success, false on failure.
         */
        bool dumpGeometry(const std::string &file) const;

        /** \brief Dumps a the tranforms of all links of a robot through a robot trajectory to a file.
         *  \param[in] path Path to output.
         *  \param[in] filename Filename to output to.
         *  \return True on success, false on failure.
         */
        bool dumpPathTransforms(const robot_trajectory::RobotTrajectory &path, const std::string &filename);

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
        void loadRobotModel();

        const std::string name_;  ///< Robot name.
        IO::Handler handler_;     ///< IO handler (namespaced with \a name_)

        std::shared_ptr<robot_model_loader::RobotModelLoader> loader_;    ///< Robot model loader.
        robot_model::RobotModelPtr model_;                                ///< Loaded robot model.
        std::map<std::string, robot_model::SolverAllocatorFn> imap_;      ///< Kinematic solver allocator map.
        kinematics_plugin_loader::KinematicsPluginLoaderPtr kinematics_;  ///< Kinematic plugin loader.

        robot_state::RobotStatePtr scratch_;  ///< Scratch robot state.

    private:
        static const std::string ROBOT_DESCRIPTION;  ///< Default robot description name.
        static const std::string ROBOT_SEMANTIC;     ///< Default robot semantic description suffix.
        static const std::string ROBOT_PLANNING;     ///< Default robot planning description suffix.
        static const std::string ROBOT_KINEMATICS;   ///< Default robot kinematics description suffix.
    };

    /** \brief Wrapper class around the planning scene and collision geometry.
     */
    class Scene
    {
    public:
        /** \brief Constructor.
         *  \param[in] robot Robot to construct planning scene for.
         */
        Scene(const RobotConstPtr &robot);

        /** \brief Copy Constructor.
         *  \param[in] scene Scene to copy.
         */
        Scene(const Scene &scene);

        /** \brief Assignment Copy Constructor.
         *  \param[in] scene Scene to copy.
         */
        void operator=(const Scene &scene);

        /** \name Getters and Setters
            \{ */

        /** \brief Get a const reference to the planning scene.
         *  \return The planning scene.
         */
        const planning_scene::PlanningScenePtr &getSceneConst() const
        {
            return scene_;
        }

        /** \brief Get a reference to the planning scene.
         *  \return The planning scene.
         */
        planning_scene::PlanningScenePtr &getScene()
        {
            return scene_;
        }

        /** \brief Get the message that describes the current planning scene.
         *  \return The planning scene message.
         */
        moveit_msgs::PlanningScene getMessage() const;

        /** \brief Get a reference to the current robot state in the planning scene.
         *  \return The planning scene robot.
         */
        robot_state::RobotState &getCurrentState();

        /** \brief Get the current allowed collision matrix of the planning scene.
         *  \return The allowed collision matrix.
         */
        collision_detection::AllowedCollisionMatrix &getACM();

        /** \} */

        /** \name Collision Object Management
            \{ */

        /** \brief Adds or updates collision object in the planning scene.
         *  If the geometry reference is the same, the collision object is updated. Otherwise, the old object
         *  named \a name is deleted and a new one is created.
         *  \param[in] name Name of object to add or update.
         *  \param[in] geometry Geometry of object.
         *  \param[in] pose Pose of object.
         */
        void updateCollisionObject(const std::string &name, const Geometry &geometry,
                                   const Eigen::Affine3d &pose);

        /** \brief Removes an object from the planning scene.
         *  \param[in] name Name of object to remove.
         */
        void removeCollisionObject(const std::string &name);

        /** \brief Get the current pose of a collision object.
         *  \param[in] name Name of object to get pose for.
         *  \return Pose of the object.
         */
        Eigen::Affine3d getObjectPose(const std::string &name);

        /** \brief Attach the named collision object \a name to the default end-effector of the robot.
         *  Only works if there is one end-effector in the system. Uses all end-effector links as allowed
         *  touch links.
         *  \param[in] name Name of collision to attach.
         *  \return True on success, false on failure.
         */
        bool attachObject(const std::string &name);

        /** \brief Attach the named collision object \a name to the link \a ee_link.
         *  \param[in] name Name of object to attach.
         *  \param[in] ee_link Link to attach object to.
         *  \param[in] touch_links Links the object is allowed to touch.
         *  \return True on success, false on failure.
         */
        bool attachObject(const std::string &name, const std::string &ee_link,
                          const std::vector<std::string> &touch_links);

        /** \brief Detach an object \a name from the robot.
         *  \param[in] name Name of collision to detach.
         *  \return True on success, false on failure.
         */
        bool detachObject(const std::string &name);

        /** \} */

        /** \name IO
            \{ */

        /** \brief Serialize the current planning scene to a YAML file.
         *  \param[in] file File to serialize planning scene to.
         *  \return True on success, false on failure.
         */
        bool toYAMLFile(const std::string &file);

        /** \brief Load a planning scene from a YAML file.
         *  \param[in] file File to load planning scene from.
         *  \return True on success, false on failure.
         */
        bool fromYAMLFile(const std::string &file);

        /** \} */

    private:
        planning_scene::PlanningScenePtr scene_;  ///< Underlying planning scene.
    };

    // TODO: Document, and fix issues of the difficulties of RViz loading.
    namespace IO
    {
        class RVIZHelper
        {
        public:
            RVIZHelper() : nh_("~")
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
