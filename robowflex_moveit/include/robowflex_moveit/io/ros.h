/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ROS_
#define ROBOWFLEX_ROS_

namespace robowflex
{
    /** \brief RAII-pattern for starting up ROS.
     */
    class ROS
    {
    public:
        /** \brief Constructor. Start-up ROS.
         *  If Boost version is greater than 1.64, `rosmaster` is started if it is not already running. A
         *  signal handler or SIGINT and SIGSEGV is installed to gracefully exit.
         *  \param[in] argc Argument count forwarded to ros::init
         *  \param[in] argv Arguments forwarded to ros::init
         *  \param[in] name Name of ROS node.
         *  \param[in] threads Threads to use for ROS spinning. If 0 no spinner is created.
         */
        ROS(int argc, char **argv, const std::string &name = "robowflex", unsigned int threads = 1);

        /** \brief Destructor. Shutdown ROS.
         */
        ~ROS();

        /** \brief Get command-line arguments without ROS parameters.
         *  \return Vector of command line arguments as strings.
         */
        std::vector<std::string> getArgs() const;

        /** \brief Waits for the process to be killed via some means (normally Ctrl-C)
         */
        void wait() const;

    private:
        int argc_;     ///< Argument count.
        char **argv_;  ///< Arguments.
    };
}  // namespace robowflex

#endif
