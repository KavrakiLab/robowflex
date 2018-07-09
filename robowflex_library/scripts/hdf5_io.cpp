/* Author: Zachary Kingston */

#include <robowflex_library/io/hdf5.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // const H5std_string filename(IO::resolvePath("~/Downloads/left_foot.hdf5"));
    // H5::H5File file(filename, H5F_ACC_RDONLY);

    // std::cout << file.getNumObjs() << std::endl;
    // std::cout << file.getObjnameByIdx(0) << std::endl;

    IO::HDF5File file("~/Downloads/left_foot.hdf5");

    for (auto obj : file.listObjects())
        std::cout << obj << std::endl;


    file.loadData("captain");

    return 0;
}
