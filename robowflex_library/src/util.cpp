/* Author: Zachary Kingston */
#include <signal.h>

#include <ros/init.h>
#include <ros/master.h>

#include <robowflex_library/macros.h>
#include <robowflex_library/util.h>

#if IS_BOOST_164
#include <boost/process.hpp>
#endif

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

        ros::start();
    }
}  // namespace

// TODO: use ros::removeROSArgs to strip ROS arguments so other arg parsers can be used in tandem.
void robowflex::startROS(int argc, char **argv, const std::string &name)
{
    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
    startup();

    signal(SIGINT, shutdown);
    signal(SIGSEGV, shutdown);
}

void robowflex::explode()
{
    raise(SIGSEGV);
}
