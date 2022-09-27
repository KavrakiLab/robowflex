/* Author: Zachary Kingston */

#include <boost/asio/ip/host_name.hpp>                        // for hostname
#include <boost/interprocess/detail/os_thread_functions.hpp>  // for process / thread IDs

#include <robowflex_moveit/io/roslog.h>
#include <robowflex_moveit/io/process.h>

using namespace robowflex;

std::string IO::runCommand(const std::string &cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        XROS_ERROR("Failed to run command `%s`!", cmd);
        return "";
    }

    while (!feof(pipe.get()))
    {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }

    return result;
}

std::string IO::getHostname()
{
    return boost::asio::ip::host_name();
}

std::size_t IO::getProcessID()
{
    return boost::interprocess::ipcdetail::get_current_process_id();
}

std::size_t IO::getThreadID()
{
    return boost::interprocess::ipcdetail::get_current_thread_id();
}
