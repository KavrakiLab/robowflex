#include <signal.h>
#include <random>

#include <boost/version.hpp>

#define IS_BOOST_164 BOOST_VERSION > 106400

#if IS_BOOST_164
#include <boost/process.hpp>
#endif

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <robowflex_library/robowflex.h>

using namespace robowflex;

namespace
{
#if IS_BOOST_164
    static boost::process::child roscore;
    static bool roscore_init{false};
#endif

    void shutdown(int sig)
    {
        // Some stuff for later
        ros::shutdown();

#if IS_BOOST_164
        if (roscore_init)
            roscore.terminate();
#endif

        exit(0);
    }

    void startup()
    {
        if (!ros::master::check())
        {
            ROS_ERROR("rosmaster is not running!");
#if IS_BOOST_164
            ROS_WARN("Booting rosmaster...");
            roscore = boost::process::child("rosmaster",                                     //
                                            boost::process::std_in.close(),                  //
                                            boost::process::std_out > boost::process::null,  //
                                            boost::process::std_err > boost::process::null   //
            );

            roscore_init = true;
#endif
        }
    }
}  // namespace

void robowflex::startROS(int argc, char **argv, const std::string &name)
{
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
    startup();

    signal(SIGINT, shutdown);
    signal(SIGSEGV, shutdown);
}

Eigen::Vector3d TF::vectorMsgToEigen(const geometry_msgs::Vector3 &msg)
{
    Eigen::Vector3d vector;
    tf::vectorMsgToEigen(msg, vector);
    return vector;
}

geometry_msgs::Vector3 TF::vectorEigenToMsg(const Eigen::Vector3d &vector)
{
    geometry_msgs::Vector3 msg;
    tf::vectorEigenToMsg(vector, msg);
    return msg;
}

Eigen::Affine3d TF::poseMsgToEigen(const geometry_msgs::Pose &msg)
{
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(msg, pose);
    return pose;
}

geometry_msgs::Pose TF::poseEigenToMsg(const Eigen::Affine3d &pose)
{
    geometry_msgs::Pose msg;
    tf::poseEigenToMsg(pose, msg);
    return msg;
}

Eigen::Quaterniond TF::quaternionMsgToEigen(const geometry_msgs::Quaternion &msg)
{
    Eigen::Quaterniond quaternion;
    tf::quaternionMsgToEigen(msg, quaternion);
    return quaternion;
}

geometry_msgs::Quaternion TF::quaternionEigenToMsg(const Eigen::Quaterniond &quaternion)
{
    geometry_msgs::Quaternion msg;
    tf::quaternionEigenToMsg(quaternion, msg);
    return msg;
}

moveit_msgs::BoundingVolume TF::getBoundingVolume(const Eigen::Affine3d &pose, const Geometry &geometry)
{
    moveit_msgs::BoundingVolume bv;

    if (geometry.isMesh())
    {
        bv.meshes.push_back(geometry.getMeshMsg());
        bv.mesh_poses.push_back(TF::poseEigenToMsg(pose));
    }
    else
    {
        bv.primitives.push_back(geometry.getSolidMsg());
        bv.primitive_poses.push_back(TF::poseEigenToMsg(pose));
    }

    return bv;
}

moveit_msgs::PositionConstraint TF::getPositionConstraint(const std::string &ee_name, const std::string &base_name,
                                                          const Eigen::Affine3d &pose, const Geometry &geometry)
{
    moveit_msgs::PositionConstraint constraint;

    constraint.header.frame_id = base_name;
    constraint.link_name = ee_name;

    constraint.constraint_region = getBoundingVolume(pose, geometry);
    constraint.weight = 1;

    return constraint;
}

moveit_msgs::OrientationConstraint TF::getOrientationConstraint(const std::string &ee_name,
                                                                const std::string &base_name,
                                                                const Eigen::Quaterniond &orientation,
                                                                const Eigen::Vector3d &tolerances)
{
    moveit_msgs::OrientationConstraint constraint;

    constraint.header.frame_id = base_name;
    constraint.link_name = ee_name;
    constraint.absolute_x_axis_tolerance = tolerances[0];
    constraint.absolute_y_axis_tolerance = tolerances[1];
    constraint.absolute_z_axis_tolerance = tolerances[2];
    constraint.orientation = TF::quaternionEigenToMsg(orientation);
    constraint.weight = 1;

    return constraint;
}

Eigen::Quaterniond TF::sampleOrientation(const Eigen::Quaterniond &orientation, const Eigen::Vector3d &tolerances)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> rng(-1.0, 1.0);

    Eigen::Quaterniond sampled = Eigen::AngleAxisd(rng(generator) * tolerances[0], Eigen::Vector3d::UnitX())    //
                                 * Eigen::AngleAxisd(rng(generator) * tolerances[1], Eigen::Vector3d::UnitY())  //
                                 * Eigen::AngleAxisd(rng(generator) * tolerances[2], Eigen::Vector3d::UnitZ());

    return orientation * sampled;
}
