/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_HDF5_
#define ROBOWFLEX_IO_HDF5_

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <numeric>

#include <boost/variant.hpp>

#include <H5Cpp.h>

#include <robowflex_library/class_forward.h>
#include <robowflex_library/io.h>

namespace robowflex
{
    namespace IO
    {
        ROBOWFLEX_CLASS_FORWARD(HDF5Data)

        class HDF5Data
        {
        public:
            template <typename T>
            HDF5Data(const T &location, const std::string &name)
              : dataset_(location.openDataSet(name))
              , space_(dataset_.getSpace())
              , type_(dataset_.getTypeClass())
              , rank_(space_.getSimpleExtentNdims())
              , dims_([&] {  //
                  hsize_t *dims = new hsize_t[rank_];
                  space_.getSimpleExtentDims(dims);
                  return dims;
              }())
              , data_([&] {  //
                  const auto &properties = getDataProperties();
                  void *data =
                      std::malloc(std::get<1>(properties) *  //
                                  std::accumulate(dims_, dims_ + rank_, 1, std::multiplies<hsize_t>()));

                  dataset_.read(data, std::get<0>(properties), space_, space_);
                  return data;
              }())
            {
            }

            ~HDF5Data()
            {
                delete dims_;
                std::free((void *)data_);
            }

            const std::vector<hsize_t> getDims() const
            {
                return std::vector<hsize_t>(dims_, dims_ + rank_);
            }

            const void *getData() const
            {
                return data_;
            }

            const std::string getStatus() const
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

        private:
            std::tuple<H5::PredType, unsigned int, std::string> getDataProperties() const
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

            const H5::DataSet dataset_;
            const H5::DataSpace space_;

            const H5T_class_t type_;
            const int rank_;
            const hsize_t *dims_;

            const void *data_;
        };

        class HDF5File
        {
        public:
            typedef boost::make_recursive_variant<
                HDF5DataPtr, std::map<std::string, boost::recursive_variant_>>::type Node;

            HDF5File(const std::string &filename) : file_(IO::resolvePath(filename), H5F_ACC_RDONLY)
            {
                for (auto obj : listObjects(file_))
                    loadData(data_, file_, obj);
            }

        private:
            template <typename T>
            const std::vector<std::string> listObjects(const T &location)
            {
                std::vector<std::string> names;
                for (hsize_t i = 0; i < location.getNumObjs(); ++i)
                    names.emplace_back(location.getObjnameByIdx(i));

                return names;
            }

            template <typename T>
            void loadData(std::map<std::string, Node> &node, const T &location, const std::string &name)
            {
                H5O_type_t type = H5O_TYPE_GROUP;
                if (!name.empty())
                    type = location.childObjType(name);

                switch (type)
                {
                    case H5O_TYPE_GROUP:
                    {
                        H5::Group group = location.openGroup(name);
                        for (auto obj : listObjects(group))
                        {
                            node[obj] = std::map<std::string, Node>();
                            loadData(boost::get<std::map<std::string, Node>>(node[obj]), group, obj);
                        }

                        break;
                    }
                    case H5O_TYPE_DATASET:
                    {
                        auto data = std::make_shared<HDF5Data>(location, name);
                        // std::cout << data->getStatus() << std::endl;

                        node[name] = std::move(data);
                        break;
                    }

                    // case H5O_TYPE_NAMED_DATATYPE:
                    // case H5O_TYPE_NTYPES:
                    default:
                        break;
                };
            }

            const H5::H5File file_;
            std::map<std::string, Node> data_;
        };

        void test();
    }  // namespace IO
}  // namespace robowflex

#endif
