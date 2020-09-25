/* Author: Zachary Kingston, Constantinos Chamzas */

#include <robowflex_library/geometry.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/random.h>

using namespace robowflex;

RobotPose TF::identity()
{
    return RobotPose::Identity();
}

RobotPose TF::createPoseXYZ(double x, double y, double z, double X, double Y, double Z)
{
    return createPoseXYZ(Eigen::Vector3d{x, y, z},  //
                         Eigen::Vector3d{X, Y, Z});
}

RobotPose TF::createPoseXYZ(const Eigen::Ref<const Eigen::Vector3d> &translation,
                            const Eigen::Ref<const Eigen::Vector3d> &rotation)
{
    RobotPose pose = RobotPose::Identity();
    pose.translation() = translation;

    pose.linear() = (Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(rotation[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ()))
                        .toRotationMatrix();

    return pose;
}

RobotPose TF::createPoseQ(double x, double y, double z, double W, double X, double Y, double Z)
{
    return createPoseQ(Eigen::Vector3d{x, y, z},  //
                       Eigen::Vector4d{W, X, Y, Z});
}

RobotPose TF::createPoseQ(const Eigen::Ref<const Eigen::Vector3d> &translation,
                          const Eigen::Ref<const Eigen::Vector4d> &rotation)
{
    return createPoseQ(translation, Eigen::Quaterniond(rotation));
}

RobotPose TF::createPoseQ(const Eigen::Ref<const Eigen::Vector3d> &translation,
                          const Eigen::Quaterniond &rotation)
{
    RobotPose pose = RobotPose::Identity();
    pose.translation() = translation;
    pose.linear() = rotation.toRotationMatrix();

    return pose;
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

    const auto &r = Eigen::Quaterniond(pose.rotation());
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

Eigen::Quaterniond TF::sampleOrientation(const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &tolerances)
{
    const auto vec = RNG::uniformVec(tolerances);
    Eigen::Quaterniond sampled = Eigen::AngleAxisd(vec[0], Eigen::Vector3d::UnitX())    //
                                 * Eigen::AngleAxisd(vec[1], Eigen::Vector3d::UnitY())  //
                                 * Eigen::AngleAxisd(vec[2], Eigen::Vector3d::UnitZ());

    return orientation * sampled;
}

Eigen::Quaterniond TF::sampleOrientationUniform(const Eigen::Vector3d &tolerances)
{
    const auto vec = RNG::uniformRPY(tolerances);
    Eigen::Quaterniond sampled = Eigen::AngleAxisd(vec[0], Eigen::Vector3d::UnitX())    //
                                 * Eigen::AngleAxisd(vec[1], Eigen::Vector3d::UnitY())  //
                                 * Eigen::AngleAxisd(vec[2], Eigen::Vector3d::UnitZ());

    return sampled;
}

Eigen::Quaterniond TF::offsetOrientation(const Eigen::Quaterniond &orientation, const Eigen::Vector3d &axis,
                                         double value)
{
    return Eigen::AngleAxisd(value, axis) * orientation;
}

Eigen::Vector3d TF::samplePositionUniform(const Eigen::Vector3d &bounds)
{
    return RNG::uniformVec(bounds);
}

Eigen::Vector3d TF::samplePositionGaussian(const Eigen::Vector3d &stddev)
{
    return RNG::gaussianVec(stddev);
}

RobotPose TF::samplePoseUniform(const Eigen::Vector3d &pos_bounds, const Eigen::Vector3d &orn_bounds)
{
    auto sampled = RobotPose::Identity();
    sampled.translation() = samplePositionUniform(pos_bounds);
    sampled.linear() = sampleOrientationUniform(orn_bounds).toRotationMatrix();

    return sampled;
}

RobotPose TF::samplePoseGaussian(const Eigen::Vector3d &pos_variances, const Eigen::Vector3d &orn_bounds)
{
    auto sampled = RobotPose::Identity();
    sampled.translation() = samplePositionUniform(pos_variances);
    sampled.linear() = sampleOrientationUniform(orn_bounds).toRotationMatrix();

    return sampled;
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

    const auto &r = Eigen::Quaterniond(tf.rotation());
    msg.transform.rotation = quaternionEigenToMsg(r);

    return msg;
}
