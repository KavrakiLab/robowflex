/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/gnuplot.h>

using namespace robowflex::IO;

#if IS_BOOST_164
namespace bp = boost::process;
#endif

GNUPlotHelper::Instance::Instance()
{
    auto path = bp::search_path("gnuplot");
    if (path.empty())
        throw Exception(1, "GNUPlot not found, please install!");

#if IS_BOOST_164
    gnuplot_ = bp::child(bp::search_path("gnuplot"), //"-persist",  //
                         bp::std_err > error_,                    //
                         // bp::std_out > output_,                //
                         bp::std_in < input_);
#else
    throw Exception(1, "GNUPlot helper not supported, Boost 1.64 and above is required!");
#endif
}

void GNUPlotHelper::Instance::write(const std::string &line)
{
#if IS_BOOST_164
    input_ << line;
#endif
}

void GNUPlotHelper::Instance::writeline(const std::string &line)
{
    write(line);
    flush();
}

void GNUPlotHelper::Instance::flush()
{
#if IS_BOOST_164
    input_ << std::endl;
#endif
}

void GNUPlotHelper::configurePlot(const PlottingOptions &options)
{
    auto in = getInstance(options.instance);

    in->writeline(log::format("set term %1% noraise", options.mode));
    in->writeline(log::format("set title \"%1%\"", options.title));

    in->writeline(log::format("set xlabel \"%1%\"", options.x.label));
    if (std::isfinite(options.x.max))
        in->writeline(log::format("set xrange [:%1%]", options.x.max));

    if (std::isfinite(options.x.min))
        in->writeline(log::format("set xrange [%1%:]", options.x.min));

    in->writeline(log::format("set ylabel \"%1%\"", options.y.label));
    if (std::isfinite(options.y.max))
        in->writeline(log::format("set yrange [:%1%]", options.y.max));

    if (std::isfinite(options.y.min))
        in->writeline(log::format("set yrange [%1%:]", options.y.min));
}

void GNUPlotHelper::timeseries(const TimeSeriesOptions &options)
{
    configurePlot(options);
    auto in = getInstance(options.instance);

    in->writeline("set datafile separator \",\"");
    in->write("plot ");

    auto n = options.points.size();

    auto it1 = options.points.begin();
    for (std::size_t i = 0; i < n; ++i, ++it1)
    {
        in->write(log::format("'%1%' using 1:2 with lines lw 2 title \"%2%\"",  //
                              (i == 0) ? "-" : "",                              //
                              it1->first));
        if (i != n - 1)
            in->write(", ");
    }

    in->flush();

    auto it2 = options.points.begin();
    for (std::size_t i = 0; i < n; ++i, ++it2)
    {
        for (const auto &point : it2->second)
            in->writeline(log::format("%1%,%2%", point.first, point.second));

        in->writeline("e");
    }
}

std::shared_ptr<GNUPlotHelper::Instance> GNUPlotHelper::getInstance(const std::string &name)
{
    if (instances_.find(name) == instances_.end())
        instances_.emplace(name, std::make_shared<Instance>());

    return instances_.find(name)->second;
}
