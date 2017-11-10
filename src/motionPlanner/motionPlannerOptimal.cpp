#include "motionPlannerOptimal.h"

std::vector<Point> MotionPlannerOptimal::search(const std::vector<std::vector<int>> &worldState, Point robotPose, Point goalPose) {
    std::vector<Point> path;
    if (worldState.empty() || worldState[0].empty()) return path;
    int rows = worldState.size(), cols = worldState[0].size();
    if (worldState[robotPose.x][robotPose.y] == 1 || worldState[goalPose.x][goalPose.y] == 1) return path;
    //openList keep the points which are reached, and closeList keep the points which are visited.
    std::list<Point *> openList;
    std::list<Point *> closeList;
    openList.push_back(&robotPose);
    Point *goalPoint = NULL;
    while (!openList.empty() && !goalPoint) {

        //Find the point which has the least F value.
        Point *curPoint = openList.front();
        for (auto point : openList) {
            if (point->F < curPoint->F) {
                curPoint = point;
            }
        }

        openList.remove(curPoint);
        closeList.push_back(curPoint);
        std::vector<std::pair<int, int>> directions({{-1, 0}, {1, 0}, {0, -1}, {0, 1}});
        for (auto d : directions) {
            Point *surPoint = new Point(curPoint->x + d.first, curPoint->y + d.second);
            if (surPoint->x < 0 || surPoint->x >= rows || surPoint->y < 0 || surPoint->y >= cols || worldState[surPoint->x][surPoint->y] == 1 || isInList(closeList, surPoint)) {
                continue;
            }
            //If current point is not in the openList, add it to the openList and compute its F, G, H.
            if (!isInList(openList, surPoint)) {
                surPoint->parent = curPoint;
                surPoint->G = 1 + curPoint->G;
                surPoint->H = abs(goalPose.x - surPoint->x) + abs(goalPose.y - surPoint->y);
                surPoint->F = surPoint->G + surPoint->H;
                openList.push_back(surPoint);
            }
            //If current point is in the openList, compute its G, if this G is smaller than old G, update G and recompute F.
            else {
                int tempG = 1 + curPoint->G;
                if (tempG < surPoint->G) {
                    surPoint->parent = curPoint;
                    surPoint->G = tempG;
                    surPoint->F = surPoint->G + surPoint->H;
                }
            }

            // If goalPose is in the openList, we arrived.
            goalPoint = isInList(openList, &goalPose);
            if (goalPoint) break;
        }

    }

    // list this path from goal to start point.
    while (goalPoint) {
        path.push_back(*goalPoint);
        goalPoint = goalPoint->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}



Point *MotionPlannerOptimal::isInList(const std::list<Point *> &lists, const Point *point) {
    for (auto p : lists) {
        if (p->x == point->x && p->y == point->y)
            return p;
    }
    return NULL;
}
