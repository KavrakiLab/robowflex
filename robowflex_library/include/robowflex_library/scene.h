/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_SCENE_
#define ROBOWFLEX_SCENE_

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <moveit_msgs/PlanningScene.h>

#include <moveit/collision_detection/collision_matrix.h>  // for collision_detection::AllowedCollisionMatrix
#include <moveit/planning_scene/planning_scene.h>         // for planning_scene::PlanningScene
#include <moveit/robot_state/robot_state.h>               // for robot_state::RobotState

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /** \endcond */

    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    /** \class robowflex::ScenePtr
        \brief A shared pointer wrapper for robowflex::Scene. */

    /** \class robowflex::SceneConstPtr
        \brief A const shared pointer wrapper for robowflex::Scene. */

    /** \brief Wrapper class around the planning scene and collision geometry.
     *
     *  The Scene class is a wrapper around _MoveIt!_'s planning_scene::PlanningScene, providing access to set
     * and manipulate collision objects, attach and detach objects to the robot, and so on. There are also
     * utilities to load and save planning scenes from YAML files (toYAMLFile() and fromYAMLFile()).
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
        const planning_scene::PlanningScenePtr &getSceneConst() const;

        /** \brief Get a reference to the planning scene.
         *  \return The planning scene.
         */
        planning_scene::PlanningScenePtr &getScene();

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

        /** \brief Set the planning scene to be the same as a message.
         *  \param[in] msg Message to use to set planning scene.
         *  \param[in] diff If true, uses the message as a diff.
         */
        void useMessage(const moveit_msgs::PlanningScene &msg, bool diff = false);

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
        void updateCollisionObject(const std::string &name, const GeometryConstPtr &geometry,
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

        /** \brief Get the pose of a particular frame in the scene.
         *  Example, use this to get the pose from /world to /base_link.
         *  \param[in] id The ID of the frame to look for.
         *  \return Pose of the object, Identity if frame is not present.
         */
        Eigen::Affine3d getFramePose(const std::string &id) const;

        /** \brief Attempts to set the collision detector plugin used by the scene to \a name.
         *  In MoveIt by default, 'Hybrid' is the only plugin defined.
         *  However, you can define your own libraries that contain classes that extend the
         *  collision_detection::CollisionPlugin class, and load them here as well.
         *  See http://moveit.ros.org/documentation/plugins/#collisionplugin for more details.
         *  \param[in] detector_name Name of the collision detection plugin.
         *  \return True if sucessful, false otherwise.
         */
        bool setCollisionDetector(const std::string &detector_name) const;

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

        /** \name Checking Collisions
            \{ */

        /** \brief Check if a robot state is in collision.
         *  \param[in] state State to check for collision.
         *  \param[in] request Optional request parameters for collision checking.
         *  \return The collision result.
         */
        collision_detection::CollisionResult
        checkCollision(const robot_state::RobotStatePtr &state,
                       const collision_detection::CollisionRequest &request = {}) const;

        /** \brief Get distance to a collision for a robot state.
         *  \param[in] state State to get distance to collision for.
         *  \return The distance of the state to collision.
         */
        double distanceToCollision(const robot_state::RobotStatePtr &state) const;

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
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(CollisionPluginLoader);
        /** \endcond */

        CollisionPluginLoaderPtr loader_;  ///< Plugin loader that sets collision detectors for the scene.
        planning_scene::PlanningScenePtr scene_;  ///< Underlying planning scene.
    };
}  // namespace robowflex

#endif
