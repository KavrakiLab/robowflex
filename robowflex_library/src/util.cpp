/* Author: Zachary Kingston */
#include <csignal>

#include <ros/init.h>
#include <ros/master.h>

#include <robowflex_library/log.h>
#include <robowflex_library/macros.h>
#include <robowflex_library/util.h>

#if IS_BOOST_164
#include <boost/process.hpp>
#endif

using namespace robowflex;

namespace
{
    static std::shared_ptr<ros::AsyncSpinner> SPINNER;

#if IS_BOOST_164
    static boost::process::child ROSCORE;
    static bool ROSCORE_INIT{false};
#endif

    void shutdown(int sig)
    {
        if (sig)
            RBX_INFO("Shutting down with signal %s.", strsignal(sig));
        else
            RBX_INFO("Shutting down.");

        if (SPINNER)
            SPINNER->stop();

        // Some stuff for later
        ros::shutdown();

#if IS_BOOST_164
        if (ROSCORE_INIT)
            ROSCORE.terminate();
#endif

        exit(0);
    }

    void startup()
    {
        if (!ros::master::check())
        {
            RBX_ERROR("rosmaster is not running!");
#if IS_BOOST_164
            RBX_WARN("Booting rosmaster...");
            ROSCORE = boost::process::child("rosmaster",                                     //
                                            boost::process::std_in.close(),                  //
                                            boost::process::std_out > boost::process::null,  //
                                            boost::process::std_err > boost::process::null   //
            );

            ROSCORE_INIT = true;
#endif
        }

        ros::start();
    }
}  // namespace

ROS::ROS(int argc, char **argv, const std::string &name, unsigned int threads) : argc_(argc), argv_(argv)
{
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
    startup();

    signal(SIGINT, shutdown);
    signal(SIGSEGV, shutdown);

    if (threads)
    {
        SPINNER.reset(new ros::AsyncSpinner(threads));
        SPINNER->start();
    }
}

ROS::~ROS()
{
    shutdown(0);
}

std::vector<std::string> ROS::getArgs() const
{
    std::vector<std::string> args;
    ros::removeROSArgs(argc_, argv_, args);

    return args;
}

void ROS::wait() const
{
    ros::waitForShutdown();
}

void robowflex::explode()
{
    raise(SIGSEGV);
}
