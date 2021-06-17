/* Author: Zachary Kingston */

#include <robowflex_library/log.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/gnuplot.h>

using namespace robowflex::IO;

#if IS_BOOST_164
namespace bp = boost::process;
#endif

GNUPlotHelper::GNUPlotHelper()
{
#if IS_BOOST_164
    gnuplot_ = bp::child(bp::search_path("gnuplot"), "-persist", bp::std_out > output_, bp::std_in < input_);
#endif
}

void GNUPlotHelper::write(const std::string &line)
{
#if IS_BOOST_164
    input_ << line;
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
    std::cout << std::endl;
#endif
}

void GNUPlotHelper::timeseries(const TimeSeriesOptions &options)
{
    writeline(log::format("set term %1% %2%", mode_, window_++));

    writeline("set datafile separator \",\"");
    writeline(log::format("set title \"%1%\"", options.title));
    writeline(log::format("set xlabel \"%1%\"", options.xlabel));
    writeline("set grid");

    write("plot ");
    for (const auto &point : options.points)
    {
        auto file = createTempDataFile(point.second);
        write(log::format("\"%1%\" using 1:2 with lines lw 2 title \"%2%\" ", file, point.first));
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
