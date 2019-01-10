/* Author: Zachary Kingston */

#include <random>
#include <eigen_conversions/eigen_msg.h>

#include <robowflex_library/geometry.h>
#include <robowflex_library/tf.h>

using namespace robowflex;

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

Eigen::Isometry3d TF::poseMsgToEigen(const geometry_msgs::Pose &msg)
{
    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(msg, pose);
    return pose;
}

geometry_msgs::Pose TF::poseEigenToMsg(const Eigen::Isometry3d &pose)
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

moveit_msgs::BoundingVolume TF::getBoundingVolume(const Eigen::Isometry3d &pose,
                                                  const GeometryConstPtr &geometry)
{
    moveit_msgs::BoundingVolume bv;

    if (geometry->isMesh())
    {
        bv.meshes.push_back(geometry->getMeshMsg());
        bv.mesh_poses.push_back(TF::poseEigenToMsg(pose));
    }
    else
    {
        bv.primitives.push_back(geometry->getSolidMsg());
        bv.primitive_poses.push_back(TF::poseEigenToMsg(pose));
    }

    return bv;
}

moveit_msgs::PositionConstraint TF::getPositionConstraint(const std::string &ee_name,
                                                          const std::string &base_name,
                                                          const Eigen::Isometry3d &pose,
                                                          const GeometryConstPtr &geometry)
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

Eigen::Quaterniond TF::sampleOrientation(const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &tolerances)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> rng(-1.0, 1.0);

    Eigen::Quaterniond sampled =
        Eigen::AngleAxisd(rng(generator) * tolerances[0], Eigen::Vector3d::UnitX())    //
        * Eigen::AngleAxisd(rng(generator) * tolerances[1], Eigen::Vector3d::UnitY())  //
        * Eigen::AngleAxisd(rng(generator) * tolerances[2], Eigen::Vector3d::UnitZ());

    return orientation * sampled;
}

Eigen::Quaterniond TF::offsetOrientation(const Eigen::Quaterniond &orientation, const Eigen::Vector3d &axis,
                                         double value)
{
    return Eigen::AngleAxisd(value, axis) * orientation;
}
