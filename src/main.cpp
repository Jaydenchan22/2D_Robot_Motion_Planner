#include <iostream>
#include <vector>
#include "planner/state.hpp"
#include "planner/environment.hpp"
#include "planner/rrt_planner.hpp"

int main() {
    std::cout << "=====================================\n";
    std::cout << "     RRT Motion Planner Test        \n";
    std::cout << "=====================================\n";

    // 1. Setup Environment with a central wall
    Environment env(20, 20);
    for (int y = 3; y <= 11; ++y) {
        env.setObstacle(7, y);
    }

    Point2D start{2.0, 7.0};
    Point2D goal{12.0, 7.0};

    // 2. Initialize RRT Planner
    RRTPlanner planner(1, 10000, 0.1); // max_iter, step_size, goal_bias
    std::cout << "Planning path with RRT...\n";
    
    std::vector<Point2D> path = planner.plan(start, goal, env);

    // 3. Print Results
    if (path.empty()) {
        std::cout << "FAILED: No path found within iteration limit!\n";
    } else {
        std::cout << "SUCCESS! RRT path found with " << path.size() << " continuous waypoints:\n";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << "  Waypoint [" << i << "]: (" << path[i].x << ", " << path[i].y << ")\n";
        }
    }

    return 0;
}