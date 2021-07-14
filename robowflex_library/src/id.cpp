/* Author: Zachary Kingston */

#include <robowflex_library/id.h>
#include <robowflex_library/io.h>

using namespace robowflex;

ID::ID() : id_(IO::generateUUID()), version_(0)
{
}

const std::string &ID::getID() const
{
    return id_;
}

std::size_t ID::getVersion() const
{
    return version_.load();
}

ID::Key ID::getKey() const
{
    return {getID(), getVersion()};
}

ID::Key ID::getNullKey()
{
    return {"NULL", 0};
}

bool ID::operator==(const ID &b) const
{
    return getID() == b.getID() and getVersion() == b.getVersion();
}

bool ID::operator==(const ID::Key &b) const
{
    return getID() == b.first and getVersion() == b.second;
}

void ID::incrementVersion()
{
    version_++;
}

bool robowflex::compareIDs(const ID &a, const ID &b)
{
    return a == b;
}

bool robowflex::compareIDs(const IDPtr &a, const IDPtr &b)
{
    return compareIDs(*a, *b);
}

bool robowflex::compareIDs(const IDConstPtr &a, const IDConstPtr &b)
{
    return compareIDs(*a, *b);
}

bool robowflex::compareIDs(const ID &a, const ID::Key &b)
{
    return a == b;
}

bool robowflex::compareIDs(const IDPtr &a, const ID::Key &b)
{
    return compareIDs(*a, b);
}

bool robowflex::compareIDs(const IDConstPtr &a, const ID::Key &b)
{
    return compareIDs(*a, b);
}

bool robowflex::compareIDs(const ID::Key &a, const ID::Key &b)
{
    return a == b;
}
