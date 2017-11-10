#include "motionPlannerRandom.h"

std::vector<Point> MotionPlannerRandom::search(const std::vector<std::vector<int>> &worldState, Point robotPose, Point goalPose) {
    std::vector<Point> path;
    if (worldState.empty() || worldState[0].empty()) return path;
    int step_num = 0, rows = worldState.size(), cols = worldState[0].size(), set_start = 0;
    if (worldState[robotPose.x][robotPose.y] == 1 || worldState[goalPose.x][goalPose.y] == 1) return path;
    int last_n_steps = sqrt(max_step_number_);
    Point cur(robotPose);
    std::unordered_set<std::pair <int, int>, boost::hash <std::pair <int, int>>> visited;
    visited.insert(std::make_pair(cur.x, cur.y));
    path.push_back(cur);
    srand (time(NULL));
    while (step_num < max_step_number_ && !(cur == goalPose)) {
        std::vector<std::pair<int, int>> directions;
        int directions_num = 0;
        int next_id = 0;
        Point next(0,0);
        // Check current ways to forward and store them in  directions.
        if (cur.x > 0 && worldState[cur.x - 1][cur.y] != 1) directions.push_back(std::make_pair(-1, 0));
        if (cur.x < rows - 1 && worldState[cur.x + 1][cur.y] != 1) directions.push_back(std::make_pair(1, 0));
        if (cur.y > 0 && worldState[cur.x][cur.y - 1] != 1) directions.push_back(std::make_pair(0, -1));
        if (cur.y < cols - 1 && worldState[cur.x][cur.y + 1] != 1) directions.push_back(std::make_pair(0, 1));
        directions_num = directions.size();
        // if there is no way to go, fail.
        if (directions_num == 0) break;

        //Never attempt to visit a cell that was visited in the last ​ sqrt(max_step_number)​ steps. We have stored the sqrt(max_step_number)​ cells in a dictionary.
        std::pair<int, int> tmp_last = directions.back();
        for (int i = 0; i < directions_num; ++i) {
            next.x = cur.x + directions[i].first;
            next.y = cur.y + directions[i].second;
            if (visited.count(std::make_pair(next.x, next.y))) {
                directions.erase(directions.begin() + i);
            }
        }

        //If all the available ways in the dictionary, we choose the last one to go.
        directions_num = directions.size();
        if (directions_num == 0) {
            directions.push_back(tmp_last);
            directions_num = 1;
        }

        next_id = rand() % directions_num;
        next.x = cur.x + directions[next_id].first;
        next.y = cur.y + directions[next_id].second;
        ++step_num;
        cur = next;
        path.push_back(next);
        visited.insert(std::make_pair(next.x, next.y));
        if (visited.size() > last_n_steps) {
            visited.erase(std::make_pair(path[set_start].x, path[set_start].y));
            ++set_start;
        }
    }

    if (!(cur == goalPose)) path.clear();
    return path;
}
