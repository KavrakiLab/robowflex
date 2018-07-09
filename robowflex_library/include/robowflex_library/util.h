/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_UTIL_
#define ROBOWFLEX_UTIL_

#include <string>
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

    /** \brief Start-up ROS.
     *  If Boost version is greater than 1.64, `rosmaster` is started if it is not already running. A signal
     *  handler for SIGINT and SIGSEGV is installed to gracefully exit.
     *  \param[in] argc Argument count forwarded to ros::init
     *  \param[in] argv Arguments forwarded to ros::init
     *  \param[in] name Name of ROS node.
     */
    void startROS(int argc, char **argv, const std::string &name = "robowflex");
}  // namespace robowflex

#endif
