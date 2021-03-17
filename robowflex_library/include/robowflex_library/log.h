/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_ROBOT_
#define ROBOWFLEX_ROBOT_

namespace robowflex
{
    namespace log
    {
        std::string formatRecurse(boost::format &f);

        template <class T, class... Args>
        std::string formatRecurse(boost::format &f, T &&t, Args &&... args)
        {
            return formatRecurse(f % std::forward<T>(t), std::forward<Args>(args)...);
        }

        template <typename... Args>
        std::string format(const std::string &fmt, Args &&... args)
        {
            boost::format f(fmt);
            return formatRecurse(f, std::forward<Args>(args)...);
        }

        void showINFORM();

        void showDEBUG();

        void showDEVMSG1();
    }
}

#endif
