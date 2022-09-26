/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_LOGGING_
#define ROBOWFLEX_LOGGING_

#include <boost/format.hpp>

/** \file Logging
 *  Functions for logging throughout Robowflex. Uses the boost::format format string syntax.
 *  See https://www.boost.org/doc/libs/1_66_0/libs/format/doc/format.html for details.
 */

namespace robowflex
{
    /** \brief Logging functions.
     */
    namespace log
    {
        /** \brief Recursion base case, return string form of formatted arguments.
         *  \param[in] f Formatter.
         *  \return String of formatted arguments.
         */
        std::string formatRecurse(boost::format &f);

        /** \brief Recursion base case, return string form of formatted arguments.
         *  \tparam[in] T Type of first argument.
         *  \tparam[in] Args format argument types.
         *  \param[in] f Formatter.
         *  \param[in] t First argument.
         *  \param[in] args Remaining format arguments.
         *  \return String of formatted arguments.
         */
        template <class T, class... Args>
        std::string formatRecurse(boost::format &f, T &&t, Args &&... args)
        {
            return formatRecurse(f % std::forward<T>(t), std::forward<Args>(args)...);
        }

        /** \brief Recursion base case, return string form of formatted arguments.
         *  \tparam[in] Args format argument types.
         *  \param[in] fmt Format string.
         *  \param[in] args Format arguments.
         *  \return String of formatted arguments.
         */
        template <typename... Args>
        std::string format(const std::string &fmt, Args &&... args)
        {
            boost::format f(fmt);
            return formatRecurse(f, std::forward<Args>(args)...);
        }
    }  // namespace log
}  // namespace robowflex

#endif
