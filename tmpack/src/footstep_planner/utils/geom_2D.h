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

}  // namespace footstep_planning
#endif
