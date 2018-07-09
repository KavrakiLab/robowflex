/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_HDF5_
#define ROBOWFLEX_IO_HDF5_

#include <string>
#include <vector>
#include <map>

#include <boost/variant.hpp>

#include <H5Cpp.h>

#include <robowflex_library/class_forward.h>

namespace robowflex
{
    namespace IO
    {
        ROBOWFLEX_CLASS_FORWARD(HDF5Data)

        class HDF5Data
        {
        public:
            template <typename T>
            HDF5Data(const T &location, const std::string &name);

            ~HDF5Data();

            const std::vector<hsize_t> getDims() const;

            const void *getData() const;

            const std::string getStatus() const;

        private:
            std::tuple<H5::PredType, unsigned int, std::string> getDataProperties() const;

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

            HDF5File(const std::string &filename);

        private:
            template <typename T>
            const std::vector<std::string> listObjects(const T &location);

            template <typename T>
            void loadData(std::map<std::string, Node> &node, const T &location, const std::string &name);

            const H5::H5File file_;
            std::map<std::string, Node> data_;
        };
    }  // namespace IO
}  // namespace robowflex

#endif
