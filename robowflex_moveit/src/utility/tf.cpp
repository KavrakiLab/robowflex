/* Author: Zachary Kingston, Constantinos Chamzas */

#include <boost/tokenizer.hpp>

#include <robowflex_moveit/utility/tf.h>
#include <robowflex_util/random.h>

using namespace robowflex;

RobotPose TF::identity()
{
    return RobotPose::Identity();
}

RobotPose TF::createPoseXYZ(double x, double y, double z)
{
    return createPoseXYZ(x, y, z, 0, 0, 0);
}

RobotPose TF::createPoseXYZ(const Eigen::Ref<const Eigen::Vector3d> &translation)
{
    RobotPose pose = RobotPose::Identity();
    pose.translation() = translation;

    return pose;
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

Eigen::Quaterniond TF::getPoseRotation(const RobotPose &pose)
{
    return Eigen::Quaterniond(pose.rotation());
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

double TF::angleNormalize(double v)
{
    return (v > constants::pi) ? constants::two_pi - v : v;
}

double TF::toDegrees(double v)
{
    double n = angleNormalize(v);
    double d = n * 180. / constants::pi;
    if (n >= 0)
        return d;

    return 360. + d;
}

double TF::toRadians(double v)
{
    if (v < 0)
        v += 360.;
    if (v >= 360.)
        v -= 360.;

    if (v <= 180.)
        return v * constants::pi / 180.;

    return -(360. - v) * constants::pi / 180.;
}

bool TF::isVecZero(const Eigen::Ref<const Eigen::VectorXd> &v, double tolerance)
{
    return v.norm() < tolerance;
}

std::vector<double> TF::stringToVec(const std::string &s, const std::string &separators)
{
    boost::char_separator<char> seps(separators.c_str());
    boost::tokenizer<boost::char_separator<char>> tokenizer(s, seps);

    std::vector<double> values;
    std::transform(tokenizer.begin(), tokenizer.end(), std::back_inserter(values),
                   [](const std::string &s) { return boost::lexical_cast<double>(s); });

    return std::vector<double>();
}
