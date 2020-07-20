/* Author: Zachary Kingston */

#include <iostream>

#include <ros/console.h>

#include <robowflex_library/io.h>
#include <robowflex_library/util.h>

#include <robowflex_movegroup/services.h>

using namespace robowflex;
using namespace robowflex::movegroup;

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    auto dirs = ros.getArgs();
    dirs.erase(dirs.begin());  // Erase program name (first arg)

    unsigned int success = 0;
    unsigned int total = 0;
    for (const auto &dir : dirs)
        for (const auto &file : IO::listDirectory(dir).second)
        {
            MoveGroupHelper::Action action;
            if (action.fromYAMLFile(file))
            {
                success += action.success;
                total++;
            }
        }

    std::cout << success << std::endl;
    std::cout << total << std::endl;

    return 0;
}
