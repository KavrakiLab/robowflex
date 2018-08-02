/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_UTIL_
#define ROBOWFLEX_UTIL_

#include <string>
#include <vector>
#include <exception>

namespace robowflex
{
    /** \brief Exception that contains a message and an error code.
     */
    class Exception : public std::exception
    {
    public:
        /** \brief Constructor.
         *  \param[in] value Error code.
         *  \param[in] message Error message.
         */
        Exception(int value, const std::string &message) : value_(value), message_(message)
        {
        }

        /** \brief Get error code.
         */
        int getValue() const
        {
            return value_;
        }

        /** \brief Get error message.
         */
        const std::string &getMessage() const
        {
            return message_;
        }

        virtual const char *what() const throw()
        {
            return message_.c_str();
        }

    protected:
        const int value_;            ///< Error code.
        const std::string message_;  ///< Error message.
    };

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

    /** \brief Trigger a SEGSEGV.
     */
    void explode();
}  // namespace robowflex

#endif
