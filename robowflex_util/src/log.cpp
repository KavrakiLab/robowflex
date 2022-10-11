/* Author: Zachary Kingston */

#include <robowflex_util/log.h>

using namespace robowflex;

std::string log::formatRecurse(boost::format &f)
{
    return boost::str(f);
}
