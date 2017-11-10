#ifndef MOTION_PLANNER_OPTIMAL_H_
#define MOTION_PLANNER_OPTIMAL_H_

#include <vector>
#include <list>
#include <algorithm>

#include "motionPlanner.h"

class MotionPlannerOptimal : public MotionPlanner {
private:
    Point *isInList(const std::list<Point *> &list, const Point *point);
public:
    MotionPlannerOptimal(){}

    /**
     * @brief search
     * @param worldState
     * @param robotPose
     * @param goalPose
     * @return
     */
    std::vector<Point> search(const std::vector<std::vector<int>>& worldState, const Point robotPose, const Point goalPose) override;
};

#endif //MOTION_PLANNER_OPTIMAL_H_
