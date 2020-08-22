/* Author: Constantinos */

#include <robowflex_library/random.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/io.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    std::ofstream angle;
    IO::createFile(angle, "angle.txt");

    std::ofstream cart;
    IO::createFile(angle, "cart.txt");

    for (int i = 0; i < 100000; i++)
    {
        const auto &no_bounds = Eigen::Vector3d{constants::pi, constants::half_pi, constants::pi};
        const auto &bounds = Eigen::Vector3d{0.5, 1, 0.2};

        const auto &rpy_no = RNG::uniformRPY(no_bounds);

        angle << rpy_no.x() << "," << rpy_no.y() << "," << rpy_no.z() << ",";

        const auto &rpy_bo = RNG::uniformRPY(bounds);

        angle << rpy_bo.x() << "," << rpy_bo.y() << "," << rpy_bo.z() << std::endl;

        const auto &tr = Eigen::Vector3d{0, 0, 1};
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
