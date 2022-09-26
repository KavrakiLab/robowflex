/* Author: Zachary Kingston, Constantinos Chamzas */

#ifndef ROBOWFLEX_IO_COLORMAP_
#define ROBOWFLEX_IO_COLORMAP_

#include <Eigen/Geometry>

namespace robowflex
{
    namespace color
    {
        /** \brief Maps a scalar s in [0, 1] to the Viridis colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] color Output color.
         */
        void viridis(double s, Eigen::Ref<Eigen::Vector4d> color);

        /** \brief Maps a scalar s in [0, 1] to the Cool-Warm colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] color Output color.
         */
        void coolwarm(double s, Eigen::Ref<Eigen::Vector4d> color);

        /** \brief Maps a scalar s in [0, 1] to the Extended Kindlmann colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] color Output color.
         */
        void extKindlmann(double s, Eigen::Ref<Eigen::Vector4d> color);

        /** \brief Maps a scalar s in [0, 1] to the Plasma colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] color Output color.
         */
        void plasma(double s, Eigen::Ref<Eigen::Vector4d> color);

        /** \brief Maps a scalar s in [0, 1] to the Turbo colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] color Output color.
         */
        void turbo(double s, Eigen::Ref<Eigen::Vector4d> color);

        /** \brief Maps a scalar s in [0, 1] to greyscale.
         *  \param[in] s Scalar to map.
         *  \param[out] color Output color.
         */
        void grayscale(double s, Eigen::Ref<Eigen::Vector4d> color);

        /** \brief Maps an RGB color to a greyscale color based on luminosity.
         *  \param[in,out] color Color to convert.
         */
        void toGrayscale(Eigen::Ref<Eigen::Vector4d> color);

        // Commonly used named colors.
        const static Eigen::Vector4d BLACK{0., 0, 0, 1};
        const static Eigen::Vector4d WHITE{1, 1, 1, 1};
        const static Eigen::Vector4d GRAY{0.5, 0.5, 0.5, 1};
        const static Eigen::Vector4d RED{1, 0, 0, 1};
        const static Eigen::Vector4d PINK{1, 0.37, 0.81, 1};
        const static Eigen::Vector4d PURPLE{0.62, 0.32, 1, 1};
        const static Eigen::Vector4d GREEN{0, 1, 0, 1};
        const static Eigen::Vector4d BLUE{0, 0, 1, 1};
        const static Eigen::Vector4d YELLOW{1, 0.88, 0.12, 1};
        const static Eigen::Vector4d ORANGE{1, 0.6, 0.06, 1};
        const static Eigen::Vector4d BROWN{0.6, 0.5, 0.38, 1};

    }  // namespace color
}  // namespace robowflex

#endif
