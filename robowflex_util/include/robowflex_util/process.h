/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_PROCESS_
#define ROBOWFLEX_PROCESS_

namespace robowflex
{
    /** \brief File and ROS Input / Output operations.
     */
    namespace IO
    {
        /** \brief Runs a command \a cmd and returns stdout as a string.
         *  \param[in] cmd Command to run.
         *  \return Contents of stdout from \a cmd, or "" on failure.
         */
        std::string runCommand(const std::string &cmd);

        /** \brief Get the hostname of the system.
         *  \return String of the hostname.
         */
        std::string getHostname();

        /** \brief Get the process ID of this process.
         *  \return The process ID.
         */
        std::size_t getProcessID();

        /** \brief Get the thread ID of the current thread.
         *  \return The thread ID.
         */
        std::size_t getThreadID();
    }  // namespace IO
}  // namespace robowflex

#endif
