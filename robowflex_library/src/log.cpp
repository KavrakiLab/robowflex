/* Author: Zachary Kingston */

#include <robowflex_library/log.h>

using namespace robowflex;

namespace
{
    void setLoggerLevel(ros::console::Level level)
    {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level))
            ros::console::notifyLoggerLevelsChanged();
    }
}  // namespace

std::string log::formatRecurse(boost::format &f)
{
    return boost::str(f);
}

void log::showUpToFatal()
{
    setLoggerLevel(ros::console::levels::Fatal);
}

void log::showUpToError()
{
    setLoggerLevel(ros::console::levels::Error);
}

void log::showUpToWarning()
{
    setLoggerLevel(ros::console::levels::Warn);
}

void log::showUpToInfo()
{
    setLoggerLevel(ros::console::levels::Info);
}

void log::showUpToDebug()
{
    setLoggerLevel(ros::console::levels::Debug);
}
