#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include <stdlib.h>
#include <iostream>
#include <vector>

struct Point {
    int x, y;
    int F, G, H;
    Point *parent;
    Point(int _x, int _y) : x(_x), y(_y), F(0), G(0), H(0), parent(NULL) {}

    bool operator== (const Point &p) {
        return p.x == x && p.y == y;
    }
};

class MotionPlanner {
public:
    MotionPlanner() {}
    MotionPlanner(int max_step_number) {}
    virtual ~MotionPlanner() {}
    virtual std::vector<Point> search(const std::vector<std::vector<int>>& worldState, const Point robotPose, const Point goalPose) = 0;
};

#endif //MOTION_PLANNER_H_
