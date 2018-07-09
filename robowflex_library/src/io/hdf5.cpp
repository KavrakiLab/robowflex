/* Author: Zachary Kingston */

#include <iostream>
#include <numeric>

#include <robowflex_library/io.h>
#include <robowflex_library/io/hdf5.h>

using namespace robowflex;

///
/// IO::HDF5Data
///

template <typename T>
IO::HDF5Data::HDF5Data(const T &location, const std::string &name)
  : dataset_(location.openDataSet(name))
  , space_(dataset_.getSpace())
  , type_(dataset_.getTypeClass())
  , rank_(space_.getSimpleExtentNdims())
  , dims_([&] {
      hsize_t *dims = new hsize_t[rank_];
      space_.getSimpleExtentDims(dims);
      return dims;
  }())
  , data_([&] {
      const auto &properties = getDataProperties();
      void *data = std::malloc(std::get<1>(properties) *  //
                               std::accumulate(dims_, dims_ + rank_, 1, std::multiplies<hsize_t>()));

      dataset_.read(data, std::get<0>(properties), space_, space_);
      return data;
  }())
{
}

template IO::HDF5Data::HDF5Data(const H5::H5File &, const std::string &);
template IO::HDF5Data::HDF5Data(const H5::Group &, const std::string &);

IO::HDF5Data::~HDF5Data()
{
    delete dims_;
    std::free((void *)data_);
}

const std::vector<hsize_t> IO::HDF5Data::getDims() const
{
    return std::vector<hsize_t>(dims_, dims_ + rank_);
}

const void *IO::HDF5Data::getData() const
{
    return data_;
}

const std::string IO::HDF5Data::getStatus() const
{
    std::stringstream ss;
    ss << "HDF5DataSet ";
    ss << "Rank: " << rank_ << ", ";
    ss << "Type: " << std::get<2>(getDataProperties()) << ", ";
    ss << "Dimensions: ";
    for (int i = 0; i < rank_; ++i)
    {
        if (i > 0)
            ss << " x ";
        ss << dims_[i];
    }

    return ss.str();
}

std::tuple<H5::PredType, unsigned int, std::string> IO::HDF5Data::getDataProperties() const
{
    switch (type_)
    {
        case H5T_INTEGER:
            return std::make_tuple(H5::PredType::NATIVE_INT, sizeof(int), "integer");
        case H5T_FLOAT:
            return std::make_tuple(H5::PredType::NATIVE_DOUBLE, sizeof(double), "double");
            // case H5T_TIME:
            // case H5T_STRING:
            // case H5T_BITFIELD:
            // case H5T_OPAQUE:
            // case H5T_COMPOUND:
            // case H5T_REFERENCE:
            // case H5T_ENUM:
            // case H5T_VLEN:
            // case H5T_ARRAY:
        default:
            break;
    }
}

///
/// IO::HDF5File
///

IO::HDF5File::HDF5File(const std::string &filename)
  : file_(IO::resolvePath(filename), H5F_ACC_RDONLY), data_(NodeMap())
{
    for (auto obj : listObjects(file_))
        loadData(data_, file_, obj);
}

const IO::HDF5DataPtr &IO::HDF5File::getData(const std::vector<std::string> &keys)
{
    NodeMap &node = boost::get<NodeMap>(data_);
    for (std::size_t i = 0; i < keys.size() - 1; ++i)
        node = boost::get<NodeMap>(node[keys[i]]);

    return boost::get<HDF5DataPtr>(node[keys.back()]);
}

namespace
{
    void getKeysHelper(std::vector<std::vector<std::string>> &keys, std::vector<std::string> key,
                       IO::HDF5File::NodeMap &node)
    {
        for (auto &element : node)
        {
            try
            {
                auto &next = boost::get<IO::HDF5File::NodeMap>(element.second);
                auto next_key = key;
                next_key.push_back(element.first);

                getKeysHelper(keys, next_key, next);
            }
            catch (std::exception &e)
            {
                keys.push_back(key);
            }
        }
    }
};  // namespace

const std::vector<std::vector<std::string>> IO::HDF5File::getKeys()
{
    NodeMap &node = boost::get<NodeMap>(data_);
    std::vector<std::vector<std::string>> keys;
    std::vector<std::string> key;

    getKeysHelper(keys, key, node);

    return keys;
}

template <typename T>
const std::vector<std::string> IO::HDF5File::listObjects(const T &location)
{
    std::vector<std::string> names;
    for (hsize_t i = 0; i < location.getNumObjs(); ++i)
        names.emplace_back(location.getObjnameByIdx(i));

    return names;
}

template const std::vector<std::string> IO::HDF5File::listObjects(const H5::H5File &);
template const std::vector<std::string> IO::HDF5File::listObjects(const H5::Group &);

template <typename T>
void IO::HDF5File::loadData(Node &node, const T &location, const std::string &name)
{
    H5O_type_t type = H5O_TYPE_GROUP;
    if (!name.empty())
        type = location.childObjType(name);

    NodeMap &map = boost::get<NodeMap>(node);
    switch (type)
    {
        case H5O_TYPE_GROUP:
        {
            map[name] = NodeMap();
            H5::Group group = location.openGroup(name);
            for (auto obj : listObjects(group))
                loadData(map[name], group, obj);

            break;
        }
        case H5O_TYPE_DATASET:
        {
            map.emplace(name, std::make_shared<HDF5Data>(location, name));
            break;
        }

        // case H5O_TYPE_NAMED_DATATYPE:
        // case H5O_TYPE_NTYPES:
        default:
            break;
    };
}

template void IO::HDF5File::loadData(Node &, const H5::H5File &, const std::string &);
template void IO::HDF5File::loadData(Node &, const H5::Group &, const std::string &);
