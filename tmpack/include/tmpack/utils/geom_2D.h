/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Andrew Wells */

#ifndef NASA_FOOTSTEP_PLANNING_UTILS_GEOM_2D_H
#define NASA_FOOTSTEP_PLANNING_UTILS_GEOM_2D_H

#include <math.h>
#include <iostream>

namespace footstep_planning {
class point_2D {
 public:
  double x, y;

  point_2D(double x, double y) {
    this->x = x;
    this->y = y;
  }

  friend std::ostream &operator<<(std::ostream &strm, const point_2D &p) {
    return strm << "<" << p.x << ", " << p.y << ">";
  }

  friend bool operator==(const point_2D &p1, const point_2D &p2) {
    return p1.x == p2.x && p1.y == p2.y;
  }

  friend bool operator!=(const point_2D &p1, const point_2D &p2) {
    return ~(p1 == p2);
  }
};

class line_segment {
 public:
  double x1, y1, x2, y2;

  line_segment(double x1, double y1, double x2, double y2) {
    this->x1 = x1;
    this->x2 = x2;
    this->y1 = y1;
    this->y2 = y2;
  }
};

double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double dist(point_2D p1, point_2D p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// returns angle from -PI to PI
double normalizeAngle(double angle) {
  if (angle < -M_PI) {
    while (angle < -M_PI) angle += 2 * M_PI;
  } else if (angle > M_PI) {
    while (angle > -M_PI) angle -= 2 * M_PI;
  }
  return angle;
}

}  // namespace footstep_planning
#endif
