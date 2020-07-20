/* Author: Zachary Kingston */

#include <iostream>
#include <robowflex_library/io/hdf5.h>

using namespace robowflex;

int main(int /*argc*/, char ** /*argv*/)
{
    // Open the HDF5 file
    IO::HDF5File file("~/Downloads/left_foot.hdf5");

    // Print out all the datasets in the file.
    auto keys = file.getKeys();
    for (auto &key : keys)
    {
        for (auto &element : key)
            std::cout << element << " / ";

        std::cout << std::endl;
    }

    // Get a dataset from the file.
    IO::HDF5DataConstPtr data = file.getData({"captain", "r2", "left_leg", "joint0", "APS1"});
    std::cout << data->getStatus() << std::endl;

    // Print out its contents
    const auto &dims = data->getDims();
    for (unsigned int i = 0; i < dims[0]; ++i)
    {
        for (unsigned int j = 0; j < dims[1]; ++j)
        {
            double v = data->get<double>({i, j});
            std::cout << v << ", ";
        }

        std::cout << std::endl;
    }

    return 0;
}
