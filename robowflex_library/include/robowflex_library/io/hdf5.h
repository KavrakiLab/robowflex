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
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(HDF5Data)
        /** \endcond */

        /** \class robowflex::HDF5DataPtr
            \brief A shared pointer wrapper for robowflex::IO::HDF5Data. */

        /** \class robowflex::HDF5DataConstPtr
            \brief A const shared pointer wrapper for robowflex::IO::HDF5Data. */

        /** \brief A container class for HDF5 DataSets loaded by an HDF5File.
         */
        class HDF5Data
        {
        public:
            /** \brief Constructor. Loads reads DataSet from file.
             *  \param[in] location Location to read data from.
             *  \param[in] name Name of object to read.
             *  \tparam H5 type to read.
             */
            template <typename T>
            HDF5Data(const T &location, const std::string &name);

            /** \brief Destructor. Cleans up all read data.
             */
            ~HDF5Data();

            /** \brief Gets the dimensions of the data. Can be used to create the array necessary to store
             *  results.
             *  \return The dimensions of the data.
             */
            const std::vector<hsize_t> getDims() const;

            /** \brief Get a pointer to the underlying data array. It is of size type[dim0][dim1]...
             *  \return A pointer to the data array.
             */
            const void *getData() const;

            /** \brief Get a string describing the data.
             *  \return A string describing the data.
             */
            const std::string getStatus() const;

            /** \brief Get the value at an index.
             *  \param[in] index The indices at each dimension.
             *  \tparam The return type of the data.
             *  \return The value at the index in the data.
             */
            template <typename T>
            const T &get(const std::vector<hsize_t> &index) const;

        private:
            /** \brief Return information about the data type of the data.
             *  \return The H5 type, the size of, and the name of the data type.
             */
            std::tuple<H5::PredType, unsigned int, std::string> getDataProperties() const;

            const H5::DataSet dataset_;  ///< Dataset being read from.
            const H5::DataSpace space_;  ///< Size of the dataset.

            const H5T_class_t type_;  ///< Type of the dataset.
            const int rank_;          ///< Rank of the dataset.
            const hsize_t *dims_;     ///< Dimensions of the dataset (rank_ dimensions)

            const void *data_;  ///< Data itself.
        };

        /** \brief An HDF5 File loaded into memory.
         */
        class HDF5File
        {
        public:
            /** \brief A recursive map that has a dictionary-like structure to store HDF5 datasets.
             */
            typedef boost::make_recursive_variant<
                HDF5DataPtr, std::map<std::string, boost::recursive_variant_>>::type Node;
            /** \brief A specific map in the recursive set.
             */
            typedef std::map<std::string, Node> NodeMap;

            /** \brief Constructor. Opens \a filename.
             *  \param[in] filename File to open.
             */
            HDF5File(const std::string &filename);

            /** \brief Get the dataset under the set of keys. Each key is applied successively.
             *  \param[in] keys The keys for the dataset to access.
             *  \return The loaded HDF5 dataset.
             */
            const HDF5DataPtr getData(const std::vector<std::string> &keys) const;

            /** \brief Gets all valid keys in the file.
             *  \return All keys in the file. Keys are vectors of strings.
             */
            const std::vector<std::vector<std::string>> getKeys() const;

        private:
            /** \brief List the objects at the HDF5 location.
             *  \param[in] location The location to search
             *  \tparam T A HDF5 object.
             */
            template <typename T>
            std::vector<std::string> listObjects(const T &location) const;

            /** \brief Loads the data in the object \a name at the HDF5 location. Recursive.
             *  \param[in] node The node to add data to.
             *  \param[in] location The location to search
             *  \param[in] name The name to search for.
             *  \tparam T A HDF5 object.
             */
            template <typename T>
            void loadData(Node &node, const T &location, const std::string &name);

            const H5::H5File file_;  ///< The loaded HDF5 file.
            Node data_;              ///< A recursive map of loaded data.
        };
    }  // namespace IO
}  // namespace robowflex

#endif
