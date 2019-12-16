/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_VISUALIZATION_
#define ROBOWFLEX_IO_VISUALIZATION_

#include <moveit/planning_interface/planning_interface.h>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    ROBOWFLEX_CLASS_FORWARD(MotionRequestBuilder);
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /** \endcond */

    namespace IO
    {
        /** \brief RVIZ visualization helper. See \ref rviz for more information.
         */
        class RVIZHelper
        {
        public:
            /** \brief Constructor. All parameters are placed under \a /name.
             *  This helper puts trajectories under /name/trajectory, scenes under /name/scene, and
             *  other markers under /name/markers. The robot description is placed under
             *  /name/robot_description.
             *  \param[in] robot Robot to model with this helper.
             *  \param[in] name Namespace to use.
             */
            RVIZHelper(const RobotConstPtr &robot, const std::string &name = "robowflex");

            /** \name Trajectories
             *  \{ */

            /** \brief Updates the trajectory being visualized.
             *  \param[in] response Planning response to visualize.
             */
            void updateTrajectory(const planning_interface::MotionPlanResponse &response);

            /** \brief Updates the trajectory being visualized to a list of trajectories.
             *  \param[in] responses Planning responses to visualize.
             */
            void updateTrajectories(const std::vector<planning_interface::MotionPlanResponse> &responses);

            /** \} */

            /** \name States
             *  \{ */

            /** \brief Visualizes a robot state.
             * \param[in] state The state of the robot to be visualized.
             */
            void visualizeState(const std::vector<double> state);

            void visualizeCurrentState();

            void visualizeStateSequence();
            /** \} */


            /** \name Scenes
             *  \{ */

            /** \brief Removes the scene being visualized.
             */
            void removeScene();

            /** \brief Updates the scene being visualized.
             *  \param[in] scene Scene to visualize. If null, removes scene by publishing empty message.
             */
            void updateScene(const SceneConstPtr &scene);

            /** \} */

            /** \name Markers
             *  \{ */

            /** \brief Easily add a spherical marker (under the name "" - the empty string) to the scene.
             *  \param[in] x The x coordinate of the sphere.
             *  \param[in] y The y coordinate of the sphere.
             *  \param[in] z The z coordinate of the sphere.
             */
            void addMarker(float x, float y, float z);

            /** \brief Add a marker to the managed list of markers. Displayed after an updateMarkers() call.
             *  \param[in] name Name of the marker.
             *  \param[in] geometry Geometry of the marker to create.
             *  \param[in] base_frame Base frame of the pose of the marker.
             *  \param[in] pose Pose of the marker.
             *  \param[in] color Color of the marker.
             */
            void addGeometryMarker(const std::string &name, const GeometryConstPtr &geometry,
                                   const std::string &base_frame, const RobotPose &pose,
                                   const Eigen::Vector4d &color = {0.2, 0.3, 0.7, 1.0});

            /** \brief Adds an arrow marker to the managed list of markers. Displayed after an updateMarkers()
             *  call.
             *  \param[in] name Name of the marker.
             *  \param[in] base_frame Base frame of the pose of the marker.
             *  \param[in] pose Pose of the marker.
             *  \param[in] color Color of the marker.
             *  \param[in] scale The scale of the marker.
             */
            void addArrowMarker(const std::string &name, const std::string &base_frame,
                                const RobotPose &pose, const Eigen::Vector4d &color,
                                const Eigen::Vector3d &scale);

            /** \brief Adds a text marker to the managed list of markers. Displayed after updateMarkers().
             *  \param[in] name Name of the marker.
             *  \param[in] text The text to display.
             *  \param[in] base_frame Base frame of the pose of the marker.
             *  \param[in] pose Pose of the marker.
             *  \param[in] height The height of the text.
             *  \param[in] color Color of the marker.
             */
            void addTextMarker(const std::string &name, const std::string &text,
                               const std::string &base_frame, const RobotPose &pose, double height,
                               const Eigen::Vector4d &color = {1, 1, 1, 1});

            /** \brief Adds the current goal of the motion request builder as a
             * set of markers in the marker array.
             *  \param[in] name Name of the marker(s).
             *  \param[in] request Request to add goal of as a marker.
             */
            void addGoalMarker(const std::string &name, const MotionRequestBuilder &request);

            /** \brief Removes a marker that was added through addMarker().
             *  \param[in] name The name of the marker to remove.
             */
            void removeMarker(const std::string &name);

            /** \brief Displays the managed list of markers.
             *  Keeps track of whether markers have already been displayed and simply need an update, and
             * removes markers removed by removeMarker().
             */
            void updateMarkers();

            /** \} */

        private:
            /** \brief Fills a marker in with some common default information.
             *  \param[out] marker Marker to fill.
             *  \param[in] base_frame Base frame of the pose of the marker.
             *  \param[in] pose Pose of the marker.
             *  \param[in] color Color of the marker.
             *  \param[in] scale The scale of the marker.
             */
            void fillMarker(visualization_msgs::Marker &marker, const std::string &base_frame,
                            const RobotPose &pose, const Eigen::Vector4d &color,
                            const Eigen::Vector3d &scale) const;

            RobotConstPtr robot_;            ///< Robot being visualized.
            ros::NodeHandle nh_;             ///< Handle for publishing.
            ros::Publisher marker_pub_;      ///< Marker publisher.
            ros::Publisher trajectory_pub_;  ///< Trajectory publisher.
            ros::Publisher scene_pub_;       ///< Scene publisher.

            std::multimap<std::string, visualization_msgs::Marker> markers_;  ///< Markers to publish.
        };
    }  // namespace IO
}  // namespace robowflex

#endif
