/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_ROBOTBROADCASTER_
#define ROBOWFLEX_IO_ROBOTBROADCASTER_

#include <thread>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/adapter.h>
#include <tf2_ros/transform_broadcaster.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(Robot);
    ROBOWFLEX_CLASS_FORWARD(Scene);
    /** \endcond */

    namespace IO
    {
        /** \brief Helper class to broadcast transform information on TF and joint states
         */
        class RobotBroadcaster
        {
        public:
            /** \brief Constructor. Sets up TF2 broadcaster.
             *  \param[in] robot Robot to broadcast.
             *  \param[in] base_frame Base frame to use for robot transforms.
             *  \param[in] name Namespace to use.
             */
            RobotBroadcaster(const RobotConstPtr &robot, const std::string &base_frame = "world",
                             const std::string &name = "robowflex");

            /** \brief Destructor.
             */
            ~RobotBroadcaster();

            /** \brief Begin publishing TF and joint information.
             */
            void start();

            /** \brief Stop publishing TF and joint information.
             */
            void stop();

            /** \brief Add a new static transform to the broadcaster.
             *  \param[in] name Name of transform (used for removal)
             *  \param[in] base Base frame of transform.
             *  \param[in] target Target frame of transform.
             *  \param[in] tf The transform from \a base to \a target.
             */
            void addStaticTransform(const std::string &name, const std::string &base,
                                    const std::string &target, const RobotPose &tf = RobotPose::Identity());

            /** \brief Remove a named static transform
             *  \param[in] name Name of transform to remove.
             */
            void removeStaticTransform(const std::string &name);

        private:
            /** \brief Send out the TF and joint information.
             */
            void update();

            RobotConstPtr robot_;                  ///< Robot being published.
            const std::string base_;               ///< Base frame to use.
            bool active_{false};                   ///< Is thread active?
            bool done_{false};                     ///< Is thread done?
            unsigned int rate_{10};                ///< Times per second to send out.
            std::unique_ptr<std::thread> thread_;  ///< Worker thread.
            ros::NodeHandle nh_;                   ///< Handle for publishing.
            tf2_ros::TransformBroadcaster tf2br_;  ///< TF2 broadcaster
            ros::Publisher state_pub_;             ///< State publisher.

            /** \brief Information for a static transform.
             */
            struct StaticTransform
            {
                std::string base;    ///< Base frame
                std::string target;  ///< Target frame
                RobotPose tf;        ///< Transform
            };

            std::map<std::string, StaticTransform> static_;  ///< Static transforms.
        };
    }  // namespace IO
}  // namespace robowflex

#endif
