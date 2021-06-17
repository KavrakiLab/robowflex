/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_GNUPLOT_
#define ROBOWFLEX_IO_GNUPLOT_

#include <robowflex_library/macros.h>

#if IS_BOOST_164
#include <boost/process.hpp>
#endif

namespace robowflex
{
    namespace IO
    {
        class GNUPlotHelper
        {
        public:
            GNUPlotHelper();

            // non-copyable
            GNUPlotHelper(GNUPlotHelper const &) = delete;
            void operator=(GNUPlotHelper const &) = delete;

            void write(const std::string &line);
            void writeline(const std::string &line);
            void flush();

            struct TimeSeriesOptions
            {
                std::string title;
                std::string xlabel{"Time (s)"};
                std::map<std::string, std::vector<std::pair<double, double>>> points;
            };

            void timeseries(const TimeSeriesOptions &options);

        private:
            std::string mode_{"qt"};
            std::size_t window_{0};
            std::string createTempDataFile(const std::vector<std::pair<double, double>> &points);

#if IS_BOOST_164
            boost::process::opstream input_;
            boost::process::ipstream output_;
            boost::process::child gnuplot_;
#endif
        };
    }  // namespace IO
}  // namespace robowflex

#endif
