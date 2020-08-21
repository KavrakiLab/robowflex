/* Author: Constantinos */

#include <robowflex_library/random.h>
#include <robowflex_library/tf.h>
// basic file operations
#include <iostream>
#include <fstream>

using namespace robowflex;

int main(int argc, char **argv)
{
    std::ofstream angle;
    angle.open("angle.txt");

    std::ofstream cart;
    cart.open("cart.txt");

    for (int i = 0; i < 100000; i++)
    {
        auto no_bounds = Eigen::Vector3d{constants::pi, constants::half_pi, constants::pi};
        auto bounds = Eigen::Vector3d{0.5, 1, 0.2};

        auto rpy_no = RND::uniformRPY(no_bounds);

        angle << rpy_no.x() << "," << rpy_no.y() << "," << rpy_no.z() << ",";

        auto rpy_bo = RND::uniformRPY(bounds);

        angle << rpy_bo.x() << "," << rpy_bo.y() << "," << rpy_bo.z() << std::endl;

        auto tr = Eigen::Vector3d{0, 0, 1};
        auto q = TF::samplePoseUniform({0, 0, 0}, no_bounds);
        auto pose_tr = (q * tr);

        cart << pose_tr.x() << "," << pose_tr.y() << "," << pose_tr.z() << ",";

        q = TF::samplePoseUniform({0, 0, 0}, bounds);
        pose_tr = (q * tr);

        cart << pose_tr.x() << "," << pose_tr.y() << "," << pose_tr.z() << std::endl;
    }

    angle.close();
    cart.close();

    return 0;
}
