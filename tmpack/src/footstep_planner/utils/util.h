#ifndef UTILS_UTIL_H
#define UTILS_UTIL_H

#include <gflags/gflags.h>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "geom_2D.h"

namespace footstep_planning
{
    DEFINE_double(scaling_factor, 66 * 2.54,
                  "The scaling factor to by applied (multiplied) to each point in "
                  "the scene file.");

    typedef boost::property<boost::edge_weight_t, double> edge_weight_property;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property,
                                  edge_weight_property>
        Graph;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef Graph::edge_descriptor Edge;

    enum class foot
    {
        right,
        left
    };
    foot operator!(foot &f)
    {
        return f == foot::right ? foot::left : foot::right;
    };

    // TODO: Pretty printing of the tree.
    void printTree(std::vector<Vertex> predecessor_array)
    {
        std::vector<std::vector<size_t>> children;
        for (size_t i = 0; i < predecessor_array.size(); i++)
        {
            children[predecessor_array[i]].push_back(i);
        }
        std::cout << "Tree: " << std::endl;
    }

    void loadScene(const std::string &file_name, std::vector<line_segment> *line_segments,
                   std::vector<std::string> *names)
    {
        std::ifstream inFile;
        inFile.open(file_name.c_str());
        if (!inFile)
        {
            std::cout << "Unable to open file: " << file_name << std::endl;
            exit(1);
        }
        for (std::string line; getline(inFile, line);)
        {
            if (line[0] == 'l')
            {
                double x1, x2, y1, y2;
                size_t comma1 = line.find(',');
                x1 = std::stod(std::string(line, 2, comma1 - 1));

                size_t semiColon = line.find(';');
                y1 = std::stod(std::string(line, comma1 + 1, semiColon - 1));
                size_t comma2 = line.find(',', semiColon);
                x2 = std::stod(std::string(line, semiColon + 1, comma2 - 1));
                y2 = std::stod(std::string(line, comma2 + 1, line.length() - 1));

                x1 *= FLAGS_scaling_factor;
                y1 *= FLAGS_scaling_factor;
                x2 *= FLAGS_scaling_factor;
                y2 *= FLAGS_scaling_factor;

                line_segments->push_back(line_segment(x1, y1, x2, y2));

                std::string s = std::string(line, comma2 + 1, line.length() - 1);
                size_t semi_colon2 = s.find(';');
                s = std::string(s, semi_colon2 + 1, s.length() - 1);
                boost::algorithm::trim(s);
                names->push_back(s);
            }
            // skip other lines
        }
    }

    int hashPoint(const double x, const double y)
    {
        return 1009 * (x * 1000) + (y * 1000);
    }

    int hashPoint(const point_2D &p)
    {
        return hashPoint(p.x, p.y);
    }

}  // namespace footstep_planning

#endif
