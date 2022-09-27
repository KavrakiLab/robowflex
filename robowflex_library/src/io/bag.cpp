/* Author: Zachary Kingston */

#include <robowflex_library/io/filesystem.h>
#include <robowflex_library/io/bag.h>

using namespace robowflex;

///
/// IO::Bag
///

IO::Bag::Bag(const std::string &file, Mode mode)
    : mode_(mode)
    , file_((mode_ == WRITE) ? file : IO::resolvePath(file))
    , bag_(file_, (mode_ == WRITE) ? rosbag::bagmode::Write : rosbag::bagmode::Read)
{
}

IO::Bag::~Bag()
{
    bag_.close();
}
