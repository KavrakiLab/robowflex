/* Author: Zachary Kingston */

#include <robowflex_util/math.h>
#include <robowflex_moveit/utility/conversions.h>
#include <robowflex_moveit/core/geometry.h>

using namespace robowflex;

Eigen::Vector3d TF::pointMsgToEigen(const geometry_msgs::Point &msg)
{
    Eigen::Vector3d vector;
    vector[0] = msg.x;
    vector[1] = msg.y;
    vector[2] = msg.z;

    return vector;
}

geometry_msgs::Point TF::pointEigenToMsg(const Eigen::Vector3d &vector)
{
    geometry_msgs::Point msg;
    msg.x = vector[0];
    msg.y = vector[1];
    msg.z = vector[2];

    return msg;
}

Eigen::Vector3d TF::vectorMsgToEigen(const geometry_msgs::Vector3 &msg)
{
    Eigen::Vector3d vector;
    vector[0] = msg.x;
    vector[1] = msg.y;
    vector[2] = msg.z;

    return vector;
}

geometry_msgs::Vector3 TF::vectorEigenToMsg(const Eigen::Vector3d &vector)
{
    geometry_msgs::Vector3 msg;
    msg.x = vector[0];
    msg.y = vector[1];
    msg.z = vector[2];

    return msg;
}

RobotPose TF::poseMsgToEigen(const geometry_msgs::Pose &msg)
{
    return RobotPose(Eigen::Translation3d(msg.position.x, msg.position.y, msg.position.z) *
                     quaternionMsgToEigen(msg.orientation));
}

geometry_msgs::Pose TF::poseEigenToMsg(const RobotPose &pose)
{
    geometry_msgs::Pose msg;

    const auto &t = pose.translation();
    msg.position.x = t.x();
    msg.position.y = t.y();
    msg.position.z = t.z();

    const auto &r = getPoseRotation(pose);
    msg.orientation = quaternionEigenToMsg(r);

    return msg;
}

Eigen::Quaterniond TF::quaternionMsgToEigen(const geometry_msgs::Quaternion &msg)
{
    return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

geometry_msgs::Quaternion TF::quaternionEigenToMsg(const Eigen::Quaterniond &quaternion)
{
    geometry_msgs::Quaternion msg;
    msg.w = quaternion.w();
    msg.x = quaternion.x();
    msg.y = quaternion.y();
    msg.z = quaternion.z();

    return msg;
}

moveit_msgs::BoundingVolume TF::getBoundingVolume(const RobotPose &pose, const GeometryConstPtr &geometry)
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
                                                          const std::string &base_name, const RobotPose &pose,
                                                          const GeometryConstPtr &geometry)
{
    moveit_msgs::PositionConstraint constraint;

    constraint.header.frame_id = base_name;
    constraint.link_name = ee_name;

    // TODO: Expose as a parameter
    constraint.target_point_offset.x = 0;
    constraint.target_point_offset.y = 0;
    constraint.target_point_offset.z = 0;

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

geometry_msgs::TransformStamped TF::transformEigenToMsg(const std::string &source, const std::string &target,
                                                        const RobotPose &tf)
{
    geometry_msgs::TransformStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = source;
    msg.child_frame_id = target;

    const auto &t = tf.translation();
    msg.transform.translation.x = t.x();
    msg.transform.translation.y = t.y();
    msg.transform.translation.z = t.z();

    const auto &r = TF::getPoseRotation(tf);
    msg.transform.rotation = quaternionEigenToMsg(r);

    return msg;
}

RobotPose TF::transformMsgToEigen(const geometry_msgs::TransformStamped &tf)
{
    RobotPose pose;
    pose.translation().x() = tf.transform.translation.x;
    pose.translation().y() = tf.transform.translation.y;
    pose.translation().z() = tf.transform.translation.z;

    pose.linear() = quaternionMsgToEigen(tf.transform.rotation).toRotationMatrix();
    return pose;
}
