/* Author: Zachary Kingston */

#include <sensor_msgs/JointState.h>

#include <robowflex_library/robot.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/io/broadcaster.h>

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
        std::string target = link->getName();

        auto msg = TF::transformEigenToMsg(source, target, tf);

        tf2br_.sendTransform(msg);
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
