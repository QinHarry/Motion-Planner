#ifndef MOTION_PLANNER_RANDOM_H_
#define MOTION_PLANNER_RANDOM_H_

#include <vector>
#include <cmath>
#include <unordered_set>
#include <boost/functional/hash.hpp>

#include "motionPlanner.h"

class MotionPlannerRandom : public MotionPlanner {
private:
    int max_step_number_;
public:
    MotionPlannerRandom(int max_step_number): max_step_number_(max_step_number) {}
    /**
     * @brief search
     * @param world_state
     * @param robot_pose
     * @param goal_pose
     * @return path
     *
     *
     */
    std::vector<Point> search(const std::vector<std::vector<int>>& world_state, const Point robot_pose, const Point goal_pose) override;
};

#endif //MOTION_PLANNER_RANDOM_H_
