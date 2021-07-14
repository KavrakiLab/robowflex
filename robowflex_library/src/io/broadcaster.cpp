/* Author: Zachary Kingston */

#include <sensor_msgs/JointState.h>

#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/tf.h>

using namespace robowflex;

namespace
{
    void arrayToVector(double *array, unsigned int n, std::vector<double> &dst)
    {
        dst = std::vector<double>(array, array + n);
    }
}  // namespace

IO::RobotBroadcaster::RobotBroadcaster(const RobotConstPtr &robot, const std::string &base_frame,
                                       const std::string &name)
  : robot_(robot), base_(base_frame), nh_("/" + name)
{
    state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
}

IO::RobotBroadcaster::~RobotBroadcaster()
{
    active_ = false;
    while (not done_)
    {
        ros::WallDuration pause(1. / rate_);
        pause.sleep();
    }
}

void IO::RobotBroadcaster::start()
{
    active_ = true;
    thread_.reset(new std::thread([&]() {
        done_ = false;

        while (active_)
        {
            update();

            ros::WallDuration pause(1. / rate_);
            pause.sleep();
        }

        done_ = true;
    }));
}

void IO::RobotBroadcaster::stop()
{
    active_ = false;
}

void IO::RobotBroadcaster::addStaticTransform(const std::string &name, const std::string &base,
                                              const std::string &target, const RobotPose &tf)
{
    if (static_.find(name) == static_.end())
    {
        StaticTransform stf;
        stf.base = base;
        stf.target = target;
        stf.tf = tf;

        static_.emplace(name, stf);
    }
    else
        RBX_ERROR("Static transform %s already in map!", name);
}

void IO::RobotBroadcaster::removeStaticTransform(const std::string &name)
{
    auto it = static_.find(name);
    if (it != static_.end())
        static_.erase(it);
    else
        RBX_ERROR("Static transform %s does not exist in map!", name);
}

void IO::RobotBroadcaster::update()
{
    const auto &state = robot_->getScratchStateConst();
    const auto &model = robot_->getModelConst();

    for (const auto &link : model->getLinkModels())
    {
        const auto &parent = link->getParentLinkModel();

        RobotPose tf =
            link->getJointOriginTransform() * state->getJointTransform(link->getParentJointModel());

        std::string source = (parent) ? parent->getName() : base_;
        const std::string &target = link->getName();

        auto msg = TF::transformEigenToMsg(source, target, tf);

        tf2br_.sendTransform(msg);
    }

    for (const auto &pair : static_)
    {
        const auto &stf = pair.second;
        auto static_msg = TF::transformEigenToMsg(stf.base, stf.target, stf.tf);
        tf2br_.sendTransform(static_msg);
    }

    unsigned int n = state->getVariableCount();

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = state->getVariableNames();

    arrayToVector(state->getVariablePositions(), n, msg.position);
    arrayToVector(state->getVariableVelocities(), n, msg.velocity);
    arrayToVector(state->getVariableEffort(), n, msg.effort);

    state_pub_.publish(msg);
}
