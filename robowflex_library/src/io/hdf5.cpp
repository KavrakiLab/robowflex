/* Author: Zachary Kingston */

#include <iostream>
#include <numeric>

#include <robowflex_library/io.h>
#include <robowflex_library/io/hdf5.h>
#include <robowflex_library/macros.h>
#include <robowflex_library/util.h>

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
      auto *dims = new hsize_t[rank_];
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

    // clang-format off
    ROBOWFLEX_PUSH_DISABLE_GCC_WARNING(-Wcast-qual)
    // clang-format on

    std::free((void *)data_);
    ROBOWFLEX_POP_GCC
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

namespace
{
    hsize_t hpow(hsize_t base, hsize_t exp)
    {
        hsize_t result = 1;
        while (exp)
        {
            if (exp & 1)
                result *= base;
            exp /= 2;
            base *= base;
        }

        return result;
    }
};  // namespace

template <typename T>
const T &IO::HDF5Data::get(const std::vector<hsize_t> &index) const
{
    if (index.size() != (unsigned int)rank_)
        throw Exception(1, "Index size must be the same as data rank!");

    const T *data = reinterpret_cast<const T *>(data_);
    unsigned int offset = 0;

    for (int i = 0; i < rank_; ++i)
        offset += hpow(dims_[i], i) * index[rank_ - (i + 1)];

    return data[offset];
}

template const int &IO::HDF5Data::get(const std::vector<hsize_t> &index) const;
template const double &IO::HDF5Data::get(const std::vector<hsize_t> &index) const;

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
            return std::make_tuple(H5::PredType::NATIVE_OPAQUE, 0, "unknown");
            break;
    }
}

///
/// IO::HDF5File
///

IO::HDF5File::HDF5File(const std::string &filename)
  : file_(IO::resolvePath(filename), H5F_ACC_RDONLY), data_(NodeMap())
{
    for (const auto &obj : listObjects(file_))
        loadData(data_, file_, obj);
}

namespace
{
    void getKeysHelper(std::vector<std::vector<std::string>> &keys, const std::vector<std::string> &key,
                       const IO::HDF5File::NodeMap &node)
    {
        for (const auto &element : node)
        {
            auto next_key = key;
            next_key.push_back(element.first);

            try
            {
                const auto &next = boost::get<IO::HDF5File::NodeMap>(element.second);
                getKeysHelper(keys, next_key, next);
            }
            catch (std::exception &e)
            {
                keys.push_back(next_key);
            }
        }
    }

    IO::HDF5DataPtr getDataHelper(const std::vector<std::string> &keys, const IO::HDF5File::NodeMap &node)
    {
        if (keys.empty())
            return nullptr;

        const auto &element = node.find(keys.front());
        if (element == node.end())
            return nullptr;

        try
        {
            const auto &next = boost::get<IO::HDF5File::NodeMap>(element->second);
            std::vector<std::string> copy(keys.begin() + 1, keys.end());
            return getDataHelper(copy, next);
        }
        catch (std::exception &e)
        {
            return boost::get<IO::HDF5DataPtr>(element->second);
        }

        return nullptr;
    }

    template <typename T>
    H5O_type_t childObjType(const T &location, const std::string &name)
    {
        H5O_info_t info;
        H5O_type_t type = H5O_TYPE_UNKNOWN;

        herr_t r = H5Oget_info_by_name(location.getId(), name.c_str(), &info, H5P_DEFAULT);

        if (r < 0)
            return type;

        switch (info.type)
        {
            case H5O_TYPE_GROUP:
            case H5O_TYPE_DATASET:
            case H5O_TYPE_NAMED_DATATYPE:
                type = info.type;
            default:
                break;
        }

        return (type);
    }
};  // namespace

const IO::HDF5DataPtr IO::HDF5File::getData(const std::vector<std::string> &keys) const
{
    const NodeMap &node = boost::get<NodeMap>(data_);
    return getDataHelper(keys, node);
}

const std::vector<std::vector<std::string>> IO::HDF5File::getKeys() const
{
    const NodeMap &node = boost::get<NodeMap>(data_);
    std::vector<std::vector<std::string>> keys;
    std::vector<std::string> key;

    getKeysHelper(keys, key, node);

    return keys;
}

template <typename T>
std::vector<std::string> IO::HDF5File::listObjects(const T &location) const
{
    std::vector<std::string> names;
    for (hsize_t i = 0; i < location.getNumObjs(); ++i)
        names.emplace_back(location.getObjnameByIdx(i));

    return names;
}

template std::vector<std::string> IO::HDF5File::listObjects(const H5::H5File &) const;
template std::vector<std::string> IO::HDF5File::listObjects(const H5::Group &) const;

template <typename T>
void IO::HDF5File::loadData(Node &node, const T &location, const std::string &name)
{
    auto &map = boost::get<NodeMap>(node);

#if ROBOWFLEX_AT_LEAST_KINETIC
    H5O_type_t type = location.childObjType(name);
#else
    H5O_type_t type = childObjType(location, name);
#endif

    switch (type)
    {
        case H5O_TYPE_GROUP:
        {
            map[name] = NodeMap();

            H5::Group group = location.openGroup(name);
            for (const auto &obj : listObjects(group))
                loadData(map[name], group, obj);

            break;
        }
        case H5O_TYPE_DATASET:
        {
            map[name] = std::make_shared<HDF5Data>(location, name);
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
