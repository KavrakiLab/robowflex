/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_GNUPLOT_
#define ROBOWFLEX_IO_GNUPLOT_

#include <robowflex_library/macros.h>
#include <robowflex_library/constants.h>

#if IS_BOOST_164
#include <boost/process.hpp>
#endif

namespace robowflex
{
    namespace IO
    {
        /** \brief Helper class to open a pipe to a GNUPlot instance for live visualization of data.
         */
        class GNUPlotHelper
        {
        public:
            using Point = std::pair<double, double>;
            using Series = std::vector<Point>;

            /** \brief Constructor. Setups up pipe to GNUPlot.
             */
            GNUPlotHelper();

            // non-copyable
            GNUPlotHelper(GNUPlotHelper const &) = delete;
            void operator=(GNUPlotHelper const &) = delete;

            /** \name Raw Input
                \{ */

            void write(const std::string &line);
            void writeline(const std::string &line);
            void flush();

            /** \} */

            /** \name Plotting
                \{ */

            struct PlottingOptions
            {
                std::string title;               ///< Title of the plot.
                std::string xlabel{"Time (s)"};  ///< X-axis label.
                std::string ylabel{""};          ///< X-axis label.
                std::string mode{"qt"};          ///< Terminal mode for GNUPlot
                std::size_t window{0};           ///< Window number. Increments with each plot created.
                double ymax = constants::nan;    ///< Upper Y-axis limit. If NaN, will auto-adjust.
                double ymin = constants::nan;    ///< Lower Y-axis limit. If NaN, will auto-adjust.
            };

            /** \brief Configure a plot using common options.
             */
            void configurePlot(const PlottingOptions &options);

            /** \brief Time series plotting options.
             */
            struct TimeSeriesOptions : PlottingOptions
            {
                std::map<std::string, Series> points;  ///< Map of line names to their time series data.
            };

            /** \brief Plot timeseries data.
             *  \param[in] options Plotting options.
             */
            void timeseries(const TimeSeriesOptions &options);

            /** \} */

        private:
            /** \brief Helper function to create a data file from XY data points.
             *  \param[in] points Vector of XY data point pairs.
             *  \return Name of temporary file created.
             */
            std::string createTempDataFile(const Series &points);

            bool debug_{false}; ///< If true, write all input to GNUPlot to stdout.

#if IS_BOOST_164
            boost::process::opstream input_;
            // boost::process::ipstream output_;
            boost::process::ipstream error_;
            boost::process::child gnuplot_;
#endif
        };
    }  // namespace IO
}  // namespace robowflex

#endif
