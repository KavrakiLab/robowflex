#include <eigen_conversions/eigen_msg.h>

#include <robowflex/robowflex.h>

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
