/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_HDF5_
#define ROBOWFLEX_IO_HDF5_

#include <string>
#include <vector>
#include <iostream>

#include <H5Cpp.h>

#include <robowflex_library/io.h>

namespace robowflex
{
    namespace IO
    {
        template <typename T>
        const std::vector<std::string> listObjects(const T &location)
        {
            std::vector<std::string> names;
            for (hsize_t i = 0; i < location.getNumObjs(); ++i)
                names.emplace_back(location.getObjnameByIdx(i));

            return names;
        }

        template <typename T>
        bool loadData(const T &location, const std::string &name);

        template <typename T>
        bool loadGroup(const T &location, const std::string &name)
        {
            H5::Group group = location.openGroup(name);

            for (auto obj : listObjects(group))
            {
                std::cout << obj << std::endl;
                loadData(group, obj);
            }

            return true;
        }

        template <typename T>
        bool loadDataSet(const T &location, const std::string &name)
        {
            H5::DataSet dataset = location.openDataSet(name);
            H5T_class_t type = dataset.getTypeClass();

            switch (type)
            {
                // case H5T_INTEGER:
                case H5T_FLOAT:
                    std::cout << "float" << std::endl;
                    break;
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

            H5::DataSpace filespace = dataset.getSpace();

            hsize_t dims[filespace.getSimpleExtentNdims()];  // dataset dimensions
            int rank = filespace.getSimpleExtentDims(dims);

            hsize_t total = 1;
            for (int i = 0; i < rank; ++i)
                total *= dims[i];

            std::stringstream ss;
            ss << "Loading Dataset of Rank: " << rank;

            void *data;
            switch (type)
            {
                    // case H5T_INTEGER:
                case H5T_FLOAT:
                {
                    data = std::malloc(total * sizeof(double));

                    H5::DataSpace mspace(rank, dims);
                    dataset.read(data, H5::PredType::NATIVE_DOUBLE, mspace, filespace);

                    ss << ", Type: Double,";

                    break;
                }
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

            ss << " Dimensions: ";
            for (int i = 0; i < rank; ++i)
            {
                if (i > 0)
                    ss << " x ";
                ss << dims[i];
            }

            std::cout << ss.str().c_str() << std::endl;

            std::free(data);

            return true;
        }

        template <typename T>
        bool loadData(const T &location, const std::string &name)
        {
            H5O_type_t type = location.childObjType(name);

            switch (type)
            {
                case H5O_TYPE_GROUP:
                    return loadGroup(location, name);
                case H5O_TYPE_DATASET:
                    return loadDataSet(location, name);
                case H5O_TYPE_NAMED_DATATYPE:
                    std::cout << "named_datatype!" << std::endl;
                    break;
                case H5O_TYPE_NTYPES:
                    std::cout << "ntypes!" << std::endl;
                    break;
                default:
                    break;
            };

            return false;
        }

        class HDF5File
        {
        public:
            HDF5File(const std::string &filename) : file_(IO::resolvePath(filename), H5F_ACC_RDONLY)
            {
            }

            void printStructure() const
            {
                for (auto obj : IO::listObjects(file_))
                {
                }
            }

            std::vector<std::string> listObjects() const
            {
                return IO::listObjects(file_);
            }

            bool loadData(const std::string &name)
            {
                return IO::loadData(file_, name);
            }

        private:
            void printStructureHelper() const
            {
            }

            const H5::H5File file_;
        };

        void test();
    }  // namespace IO
}  // namespace robowflex

#endif
