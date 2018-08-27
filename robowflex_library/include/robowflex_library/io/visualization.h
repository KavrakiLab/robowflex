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
    ROBOWFLEX_CLASS_FORWARD(Geometry);
    /** \endcond */

    namespace IO
    {
        /** \brief RVIZ visualization helper.
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

            /** \brief Updates the trajectory being visualized.
             *  \param[in] response Planning response to visualize.
             */
            void updateTrajectory(const planning_interface::MotionPlanResponse &response);

            /** \brief Updates the trajectory being visualized to a list of trajectories.
             *  \param[in] responses Planning responses to visualize.
             */
            void updateTrajectories(const std::vector<planning_interface::MotionPlanResponse> &responses);

            /** \brief Updates the scene being visualized.
             *  \param[in] scene Scene to visualize.
             */
            void updateScene(const SceneConstPtr &scene);

            /** \brief TODO
             */
            void updateMarkers();

            void addMarker(float x, float y, float z);

            /** \brief Add a marker to the managed list of markers. Displayed after an updateMarkers() call.
             *  \param[in] name Name of the marker.
             *  \param[in] geometry Geometry of the marker to create.
             *  \param[in] base_frame Base frame of the pose of the marker.
             *  \param[in] pose Pose of the marker.
             *  \param[in] color Color of the marker.
             */
            void addGeometryMarker(const std::string &name, const GeometryConstPtr &geometry,
                                   const std::string &base_frame, const Eigen::Affine3d &pose,
                                   const Eigen::Vector4d &color = {0.2, 0.3, 0.7, 1.0});

            /** \brief Removes a marker that was added through addMarker().
             *  \param[in] name The name of the marker to remove.
             */
            void removeMarker(const std::string &name);

        private:
            RobotConstPtr robot_;            ///< Robot being visualized.
            ros::NodeHandle nh_;             ///< Handle for publishing.
            ros::Publisher marker_pub_;      ///< Marker publisher.
            ros::Publisher trajectory_pub_;  ///< Trajectory publisher.
            ros::Publisher scene_pub_;       ///< Scene publisher.

            std::map<std::string, visualization_msgs::Marker> markers_;  ///< Markers to publish.
        };
    }  // namespace IO
}  // namespace robowflex

#endif
