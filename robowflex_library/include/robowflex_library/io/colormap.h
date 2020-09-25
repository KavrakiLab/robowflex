/* Author: Zachary Kingston, Constantinos Chamzas */

#ifndef ROBOWFLEX_IO_COLORMAP_
#define ROBOWFLEX_IO_COLORMAP_

namespace robowflex
{
    namespace color
    {
        /** \brief Maps a scalar s in [0, 1] to the Viridis colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] r Red value in [0, 1]
         *  \param[out] g Green value in [0, 1]
         *  \param[out] b Blue value in [0, 1]
         */
        void viridis(double s, double &r, double &g, double &b);

        /** \brief Maps a scalar s in [0, 1] to the Cool-Warm colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] r Red value in [0, 1]
         *  \param[out] g Green value in [0, 1]
         *  \param[out] b Blue value in [0, 1]
         */
        void coolwarm(double s, double &r, double &g, double &b);

        /** \brief Maps a scalar s in [0, 1] to the Extended Kindlmann colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] r Red value in [0, 1]
         *  \param[out] g Green value in [0, 1]
         *  \param[out] b Blue value in [0, 1]
         */
        void extKindlmann(double s, double &r, double &g, double &b);

        /** \brief Maps a scalar s in [0, 1] to the Plasma colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] r Red value in [0, 1]
         *  \param[out] g Green value in [0, 1]
         *  \param[out] b Blue value in [0, 1]
         */
        void plasma(double s, double &r, double &g, double &b);

        /** \brief Maps a scalar s in [0, 1] to the Turbo colormap.
         *  \param[in] s Scalar to map.
         *  \param[out] r Red value in [0, 1]
         *  \param[out] g Green value in [0, 1]
         *  \param[out] b Blue value in [0, 1]
         */
        void turbo(double s, double &r, double &g, double &b);

        /** \brief Maps a scalar s in [0, 1] to greyscale.
         *  \param[in] s Scalar to map.
         *  \param[out] r Red value in [0, 1]
         *  \param[out] g Green value in [0, 1]
         *  \param[out] b Blue value in [0, 1]
         */
        void grayscale(double s, double &r, double &g, double &b);

        /** \brief Maps an RGB color to a greyscale color based on luminosity.
         *  \param[in,out] r Red value in [0, 1]
         *  \param[in,out] g Green value in [0, 1]
         *  \param[in,out] b Blue value in [0, 1]
         */
        void toGrayscale(double &r, double &g, double &b);

        /** \brief Convert a RGB color to HSV.
         *  \param[in] r Red value in [0, 1]
         *  \param[in] g Green value in [0, 1]
         *  \param[in] b Blue value in [0, 1]
         *  \param[out] h Hue value in [-pi, pi]
         *  \param[out] s Saturation value in [0, 1]
         *  \param[out] v Value in [0, 1]
         */
        void rgb2hsv(double r, double g, double b, double &h, double &s, double &v);

        /** \brief Convert a RGB color to HSV.
         *  \param[in] h Hue value in [-pi, pi]
         *  \param[in] s Saturation value in [0, 1]
         *  \param[in] v Value in [0, 1]
         *  \param[out] r Red value in [0, 1]
         *  \param[out] g Green value in [0, 1]
         *  \param[out] b Blue value in [0, 1]
         */
        void hsv2rgb(double h, double s, double v, double &r, double &g, double &b);
    }  // namespace color
}  // namespace robowflex

#endif
