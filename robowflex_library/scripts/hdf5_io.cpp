/* Author: Zachary Kingston */

#include <iostream>
#include <robowflex_library/io/hdf5.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    IO::HDF5File file("~/Downloads/left_foot.hdf5");

    auto keys = file.getKeys();
    for (auto &key : keys)
    {
        for (auto &element : key)
            std::cout << element << " / ";

        std::cout << std::endl;
    }

    IO::HDF5DataConstPtr data = file.getData({"captain", "r2", "left_leg", "joint0", "APS1"});
    std::cout << data->getStatus() << std::endl;

    return 0;
}
