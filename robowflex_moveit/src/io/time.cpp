#include <thread>
#include <robowflex_moveit/io/time.h>

using namespace robowflex;

boost::posix_time::ptime IO::getDate()
{
    return boost::posix_time::microsec_clock::local_time();
}

double IO::getSeconds(boost::posix_time::ptime start, boost::posix_time::ptime finish)
{
    auto duration = finish - start;
    return duration.total_microseconds() / 1000000.;
}

void IO::threadSleep(double seconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long int>(seconds * 1000)));
}
