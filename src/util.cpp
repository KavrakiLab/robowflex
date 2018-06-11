#include <eigen_conversions/eigen_msg.h>

#include "robowflex.h"

using namespace robowflex;

Eigen::Vector3d TF::vectorMsgToEigen(const geometry_msgs::Vector3& msg)
{
    Eigen::Vector3d vector;
    tf::vectorMsgToEigen(msg, vector);
    return vector;
}

geometry_msgs::Vector3 TF::vectorEigenToMsg(const Eigen::Vector3d& vector)
{
    geometry_msgs::Vector3 msg;
    tf::vectorEigenToMsg(vector, msg);
    return msg;
}

Eigen::Affine3d TF::poseMsgToEigen(const geometry_msgs::Pose& msg)
{
    Eigen::Affine3d pose;
    tf::poseMsgToEigen(msg, pose);
    return pose;
}

geometry_msgs::Pose TF::poseEigenToMsg(const Eigen::Affine3d& pose)
{
    geometry_msgs::Pose msg;
    tf::poseEigenToMsg(pose, msg);
    return msg;
}

Eigen::Quaterniond TF::quaternionMsgToEigen(const geometry_msgs::Quaternion& msg)
{
    Eigen::Quaterniond quaternion;
    tf::quaternionMsgToEigen(msg, quaternion);
    return quaternion;
}

geometry_msgs::Quaternion TF::quaternionEigenToMsg(const Eigen::Quaterniond& quaternion)
{
    geometry_msgs::Quaternion msg;
    tf::quaternionEigenToMsg(quaternion, msg);
    return msg;
}
