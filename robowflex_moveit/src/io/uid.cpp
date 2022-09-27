#include <unordered_map>  // for std::hash

#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>             // for UUID generation
#include <boost/uuid/uuid_generators.hpp>  // for UUID generation
#include <boost/uuid/uuid_io.hpp>          // for UUID generation

#include <robowflex_moveit/io/uid.h>

using namespace robowflex;

std::string IO::generateUUID()
{
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();

    std::string s = boost::lexical_cast<std::string>(u);

    std::replace(s.begin(), s.end(), '-', '_');

    return s;
}

std::size_t IO::hashString(const std::string &string)
{
    return std::hash<std::string>{}(string);
}
