#include <gflags/gflags.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "calc_footsteps.cpp"
#include "utils/geom_2D.h"
#include "utils/util.h"

using namespace footstep_planning;
int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  foot start_foot = foot::left;
  std::vector<line_segment> line_segments;
  std::vector<std::string> line_names;
  size_t start = 0, goal = 0;

  point_2D torso_goal_pose(0, 0);

  if (argc > 1) {
    if (argc == 3) {
      loadScene("../scenes/iss.txt", &line_segments, &line_names);

      std::cout << "str: " << argv[1] << " " << std::string(argv[1]).find(".")
                << std::endl;
      if (std::string(argv[1]).find(".") != std::string::npos) {
        start = 9;
        double x = std::stod(std::string(argv[1]));
        double y = std::stod(std::string(argv[2]));
        torso_goal_pose = point_2D(x, y);
        goal = -1;
      } else {
        start = atoi(argv[1]);
        goal = atoi(argv[2]);
      }
    }
    if (argc >= 4) {
      loadScene(argv[1], &line_segments, &line_names);
      start = atoi(argv[2]);
      goal = atoi(argv[3]);
    }
    if (argc == 5) {
      if (argv[4][0] == 'r') {
        start_foot = foot::right;
      } else {
        start_foot = foot::left;
      }
    }
  } else {
    loadScene("../scenes/iss.txt", &line_segments, &line_names);
    start = 9;
    goal = 17;
  }

  std::vector<point_2D> points;
  // store a map of points and their names
  boost::unordered_map<int, std::string> point_to_name_map;

  // we only use the end points and the centers
  for (size_t i = 0; i < line_segments.size(); i++) {
    auto l = line_segments[i];
    // std::string i_s = std::to_string(i);
    std::string i_s = line_names[i];
    points.push_back(point_2D(l.x1, l.y1));
    point_to_name_map.insert(std::make_pair(hashPoint(l.x1, l.y1), i_s + " l"));
    points.push_back(point_2D(l.x2, l.y2));
    point_to_name_map.insert(std::make_pair(hashPoint(l.x2, l.y2), i_s + " h"));
    points.push_back(point_2D((l.x1 + l.x2) / 2, (l.y1 + l.y2) / 2));
    point_to_name_map.insert(std::make_pair(
        hashPoint((l.x1 + l.x2) / 2, (l.y1 + l.y2) / 2), i_s + " c"));
  }

  std::cout << "start: " << start << "; goal: " << goal << std::endl;

  // if we have the original torso point and no specified indices
  if ((torso_goal_pose.x == 0 && torso_goal_pose.y == 0) &&
      (start >= points.size() || goal >= points.size())) {
    std::cout << "Invalid start or goal" << std::endl;
    exit(1);
  }

  point_2D p = points[start];
  FootstepPlanner plnr;
  plnr.buildGraph(points);
  std::vector<std::vector<point_2D>> steps;
  std::list<std::list<point_2D>> paths;

  if (goal == -1) {
    steps = plnr.calculateFootPlacementsForTorso(points, p, torso_goal_pose, 0,
                                                 start_foot);
  } else {
    std::cout<<"we are calculating based on a goal point index "<<start<<", "<<goal<<std::endl;
    point_2D p2 = points[goal];
    steps.push_back(plnr.calculateFootPlacements(points, p, p2, start_foot, !start_foot));
    //steps = plnr.calculateFootPlacementsForTorso(points, p, p2, 0, start_foot);
    paths = plnr.calculateNFootPlacements(points, p, p2, start_foot, !start_foot, 5);
  }


  for(size_t i = 0; i < points.size(); i++) {
    if(points[i] == point_2D(0,0)) {
      std::cout<<"We have the <0,0> point!"<<std::endl;
    } else {
      std::cout<<points[i]<<std::endl;
    }
  }

  for(auto pth : paths) {
    std::cout<<"========================Step points: "<<std::endl;
    for(auto pt : pth) {
      if(pt == point_2D(0,0)) {
        std::cout<<"We have the <0,0> point!"<<std::endl;
      } else {
        std::cout<<pt<<std::endl;
      }
    }    
  }
  std::cout<<"========================Step points: "<<std::endl;
  for(size_t i = 0; i < steps[0].size(); i++) {
    if(steps[0][i] == point_2D(0,0)) {
      std::cout<<"We have the <0,0> point!"<<std::endl;
    } else {
      std::cout<<steps[0][i]<<std::endl;
    }
  }


  std::cout << "Footsteps( "<<steps[0].size()<<" ):" << std::endl;
  foot curr_foot = start_foot;
  std::ofstream output_file;
  output_file.open("output_plan.txt");
  for (size_t i = 0; i < steps[0].size(); i++) {
    auto p = steps[0][i];
    boost::unordered_map<int, std::string>::const_iterator got =
        point_to_name_map.find(hashPoint(p));

    std::string s1;
    switch (curr_foot) {
      case foot::left:
        s1 = "Left";
        break;
      case foot::right:
        s1 = "Right";
        break;
    }

    std::string s2;
    if(got == point_to_name_map.end()) {
      std::cout<<"We didn't find the point: "<<p<<std::endl;
      std::cout<<"Exiting"<<std::endl;
      return -1;
    }

    switch (got->second.c_str()[got->second.size() - 1]) {
      case 'l':
        s2 = "Low";
        break;
      case 'c':
        s2 = "Center";
        break;
      case 'h':
        s2 = "High";
        break;
      default:
        std::cout << "we found a bad handrail position character "
                  << got->second << std::endl;
    }

    // the last 2 characters are a space and 'l', 'r' or 'c'
    std::string rail_name = got->second.substr(0, got->second.length() - 2);
    std::cout << p << ": " << rail_name << " " << s1 << " Foot Approach " << s2
              << std::endl;
    // Note that current_foot works out because we don't write the first
    // rail For the yaml file, start_config is the foot that is locked down
    // within this program current_foot is the foot that is moving
    // skipping the first rail makes these match
    if (i > 0) {  // we don't need the first rail
      output_file << "- " << rail_name << " " << s1 << " Foot Approach " << s2
                  << std::endl;
      if (curr_foot == foot::right) {  // these are swapped now
        output_file << "- Handrail Right Foot Grab" << std::endl;
        if (i < steps.size() - 1) {
          output_file << "- Handrail Left Foot Release" << std::endl;
        }
      } else {
        output_file << "- Handrail Left Foot Grab" << std::endl;
        if (i < steps.size() - 1) {
          output_file << "- Handrail Right Foot Release" << std::endl;
        }
      }
    }

    curr_foot = !curr_foot;
  }

  output_file.close();

  return 0;
}
