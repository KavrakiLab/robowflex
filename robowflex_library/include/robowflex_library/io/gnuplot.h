/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_IO_GNUPLOT_
#define ROBOWFLEX_IO_GNUPLOT_

#include <robowflex_library/macros.h>
#include <robowflex_library/constants.h>
#include <robowflex_library/benchmarking.h>

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
            using Values = std::vector<double>;

            GNUPlotHelper() = default;

            // non-copyable
            GNUPlotHelper(GNUPlotHelper const &) = delete;
            void operator=(GNUPlotHelper const &) = delete;

            /** \name Plotting
                \{ */

            struct PlottingOptions
            {
                std::string instance{"default"};

                struct Axis
                {
                    std::string label;            ///< Axis label.
                    double max = constants::nan;  ///< Upper axis limit. If NaN, will auto-adjust.
                    double min = constants::nan;  ///< Lower axis limit. If NaN, will auto-adjust.
                };

                std::string title;       ///< Title of the plot.
                std::string mode{"qt"};  ///< Terminal mode for GNUPlot

                Axis x;  ///< X-axis parameters.
                Axis y;  ///< Y-axis parameters.
            };

            /** \brief Configure a plot using common options.
             */
            void configurePlot(const PlottingOptions &options);

            /** \brief Time series plotting options.
             */
            struct TimeSeriesOptions : PlottingOptions
            {
                std::map<std::string, Series> points;  ///< Map of names to time series data.
            };

            /** \brief Plot timeseries data.
             *  \param[in] options Plotting options.
             */
            void timeseries(const TimeSeriesOptions &options);

            /** \brief Box plotting options.
             */
            struct BoxPlotOptions : PlottingOptions
            {
                bool outliers{true};
                bool sorted{true};
                std::map<std::string, Values> values;  ///< Map of names to data.
            };

            /** \brief Plot box data.
             *  \param[in] options Plotting options.
             */
            void boxplot(const BoxPlotOptions &options);

            /** \} */

        private:
            class Instance
            {
            public:
                /** \brief Constructor. Setups up pipe to GNUPlot.
                 */
                Instance();

                /** \name Raw Input
                    \{ */

                void write(const std::string &line);
                void writeline(const std::string &line);
                void flush();

                /** \} */

            private:
                // non-copyable
                Instance(Instance const &) = delete;
                void operator=(Instance const &) = delete;

                bool debug_{false};

#if IS_BOOST_164
                boost::process::opstream input_;
                // boost::process::ipstream output_;
                boost::process::ipstream error_;
                boost::process::child gnuplot_;
#endif
            };

            /** \brief Get the named GNUPlot instance.
             *  \param[in] name Name of instance.
             *  \return The instance.
             */
            std::shared_ptr<Instance> getInstance(const std::string &name);
            std::map<std::string, std::shared_ptr<Instance>> instances_;  ///< Map of open GNUPlot instances
        };

        /** \brief Helper class to plot a real metric as a box plot using GNUPlot from benchmarking data.
         */
        class GNUPlotPlanDataSetOutputter : public PlanDataSetOutputter
        {
        public:
            /** \brief Constructor.
             */
            GNUPlotPlanDataSetOutputter(const std::string &metric);

            /** \brief Destructor.
             */
            ~GNUPlotPlanDataSetOutputter() override;

            /** \brief Visualize results.
             *  \param[in] results Results to visualize.
             */
            void dump(const PlanDataSet &results) override;

        private:
            const std::string metric_;
            GNUPlotHelper helper_;
        };
    }  // namespace IO
}  // namespace robowflex

#endif
