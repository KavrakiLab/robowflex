/* Author: Zachary Kingston */

#include <robowflex_library/util.h>
#include <robowflex_library/log.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/gnuplot.h>

using namespace robowflex::IO;

#if IS_BOOST_164
namespace bp = boost::process;
#endif

GNUPlotHelper::GNUPlotHelper()
{
    auto path = bp::search_path("gnuplot");
    if (path.empty())
        throw Exception(1, "GNUPlot not found, please install!");

#if IS_BOOST_164
    gnuplot_ = bp::child(bp::search_path("gnuplot"), "-persist",  //
                         bp::std_err > error_,                    //
                         // bp::std_out > output_,                //
                         bp::std_in < input_);
#else
    throw Exception(1, "GNUPlot helper not supported, Boost 1.64 and above is required!");
#endif
}

void GNUPlotHelper::write(const std::string &line)
{
#if IS_BOOST_164
    input_ << line;

    if (debug_)
        std::cout << line;
#endif
}

void GNUPlotHelper::writeline(const std::string &line)
{
    write(line);
    flush();
}

void GNUPlotHelper::flush()
{
#if IS_BOOST_164
    input_ << std::endl;

    if (debug_)
        std::cout << std::endl;
#endif
}

void GNUPlotHelper::configurePlot(const PlottingOptions &options)
{
    writeline(log::format("set term %1% %2%", options.mode, options.window));
    writeline(log::format("set title \"%1%\"", options.title));
    writeline(log::format("set xlabel \"%1%\"", options.xlabel));
    writeline(log::format("set ylabel \"%1%\"", options.ylabel));

    if (std::isfinite(options.ymax))
        writeline(log::format("set yrange [:%1%]", options.ymax));

    if (std::isfinite(options.ymin))
        writeline(log::format("set yrange [%1%:]", options.ymin));
}

void GNUPlotHelper::timeseries(const TimeSeriesOptions &options)
{
    configurePlot(options);

    writeline("set datafile separator \",\"");
    write("plot ");

    auto n = options.points.size();
    auto it = options.points.begin();

    for (std::size_t i = 0; i < n; ++i)
    {
        auto file = createTempDataFile(it->second);
        write(log::format("\"%1%\" using 1:2 with lines lw 2 title \"%2%\"", file, it->first));
        if (i != n - 1)
            write(", ");

        it++;
    }

    flush();
}

std::string GNUPlotHelper::createTempDataFile(const std::vector<std::pair<double, double>> &points)
{
    std::ofstream out;
    auto ret = createTempFile(out);

    for (const auto &point : points)
        out << point.first << "," << point.second << std::endl;

    out.close();
    return ret;
}
