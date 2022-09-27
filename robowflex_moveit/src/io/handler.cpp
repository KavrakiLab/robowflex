/* Author: Zachary Kingston */

#include <robowflex_moveit/io/filesystem.h>
#include <robowflex_moveit/io/handler.h>

using namespace robowflex;

///
/// IO::Handler
///

// Static ID for all handlers
const std::string IO::Handler::UUID(IO::generateUUID());

namespace
{
    class XmlRpcValueCreator : public XmlRpc::XmlRpcValue
    {
    public:
        static XmlRpcValueCreator createArray(const std::vector<XmlRpcValue> &values)
        {
            XmlRpcValueCreator ret;
            ret._type = TypeArray;
            ret._value.asArray = new ValueArray(values);

            return ret;
        }

        static XmlRpcValueCreator createStruct(const std::map<std::string, XmlRpcValue> &members)
        {
            XmlRpcValueCreator ret;
            ret._type = TypeStruct;
            ret._value.asStruct = new std::map<std::string, XmlRpcValue>(members);
            return ret;
        }
    };

    XmlRpc::XmlRpcValue YAMLToXmlRpc(const YAML::Node &node)
    {
        if (node.Type() != YAML::NodeType::Scalar)
        {
            switch (node.Type())
            {
                case YAML::NodeType::Map:
                {
                    std::map<std::string, XmlRpc::XmlRpcValue> members;
                    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                        members[it->first.as<std::string>()] = YAMLToXmlRpc(it->second);

                    return XmlRpcValueCreator::createStruct(members);
                }
                case YAML::NodeType::Sequence:
                {
                    std::vector<XmlRpc::XmlRpcValue> values;
                    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                        values.push_back(YAMLToXmlRpc(*it));

                    return XmlRpcValueCreator::createArray(values);
                }
                default:
                    throw std::runtime_error("Unknown non-scalar node type in YAML");
            }
        }

        if (node.Tag() == "!!int")
            return XmlRpc::XmlRpcValue(node.as<int>());

        if (node.Tag() == "!!float")
            return XmlRpc::XmlRpcValue(node.as<double>());

        if (node.Tag() == "!!bool")
            return XmlRpc::XmlRpcValue(node.as<bool>());

        try
        {
            return XmlRpc::XmlRpcValue(node.as<bool>());
        }
        catch (YAML::Exception &)
        {
        }

        try
        {
            return XmlRpc::XmlRpcValue(node.as<int>());
        }
        catch (YAML::Exception &)
        {
        }

        try
        {
            return XmlRpc::XmlRpcValue(node.as<double>());
        }
        catch (YAML::Exception &)
        {
        }

        try
        {
            return XmlRpc::XmlRpcValue(node.as<std::string>());
        }
        catch (YAML::Exception &)
        {
        }

        throw std::runtime_error("Unknown node value in YAML");
    }
}  // namespace

IO::Handler::Handler(const std::string &name)
  : name_(name), namespace_("robowflex_" + UUID + "/" + name_), nh_(namespace_)
{
}

IO::Handler::Handler(const IO::Handler &handler, const std::string &name)
  : name_(handler.getName()), namespace_(handler.getNamespace()), nh_(handler.getHandle(), name)
{
}

IO::Handler::~Handler()
{
    for (const auto &key : params_)
        nh_.deleteParam(key);
}

void IO::Handler::loadYAMLtoROS(const YAML::Node &node, const std::string &prefix)
{
    switch (node.Type())
    {
        case YAML::NodeType::Map:
        {
            const std::string fixed_prefix = (prefix.empty()) ? "" : (prefix + "/");
            for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
                loadYAMLtoROS(it->second, fixed_prefix + it->first.as<std::string>());

            break;
        }
        case YAML::NodeType::Sequence:
        case YAML::NodeType::Scalar:
        {
            setParam(prefix, YAMLToXmlRpc(node));
            break;
        }
        default:
            throw std::runtime_error("Unknown node type in YAML");
    }
}

bool IO::Handler::hasParam(const std::string &key) const
{
    return nh_.hasParam(key);
}

const ros::NodeHandle &IO::Handler::getHandle() const
{
    return nh_;
}

const std::string &IO::Handler::getName() const
{
    return name_;
}

const std::string &IO::Handler::getNamespace() const
{
    return namespace_;
}
