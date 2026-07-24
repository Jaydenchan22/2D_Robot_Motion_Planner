#include <iostream>
#include <vector>
#include "planner/state.hpp"
#include "planner/environment.hpp"
#include "planner/astar_planner.hpp"

int main() {
    std::cout << "=====================================\n";
    std::cout << "   2D Motion Planner Initialized     \n";
    std::cout << "=====================================\n";

    // 1. Setup a 10x10 environment with a wall
    Environment env(20,30);
    for (int y = 0; y <= 14; ++y) {
        env.setObstacle(4, y);
    }
    for (int x = 2; x <= 15; ++x) {
        env.setObstacle(x, 14);
    }

    // 2. Define Start (Left side) and Goal (Right side)
    Point2D start{1.0, 5.0};
    Point2D goal{13.0, 5.0};

    // 3. Run the A* Planner
    AStarPlanner planner;
    std::cout << "\nPlanning path with A*...\n";
    std::vector<Point2D> path = planner.plan(start, goal, env);

    // 4. Output the results
    if (path.empty()) {
        std::cout << "FAILED: No valid path found around obstacles!\n";
    } else {
        std::cout << "SUCCESS! Path found with " << path.size() << " waypoints.\n";
        
        // Render map with path overlay
        std::cout << "\n--- Path Visualization (* = Robot Path) ---\n";
        for (int y = 0; y < env.getHeight(); ++y) {
            for (int x = 0; x < env.getWidth(); ++x) {
                // Check if this cell is part of the path
                bool is_path = false;
                for (const auto& pt : path) {
                    if (static_cast<int>(pt.x) == x && static_cast<int>(pt.y) == y) {
                        is_path = true;
                        break;
                    }
                }

                if (static_cast<int>(start.x) == x && static_cast<int>(start.y) == y) {
                    std::cout << "(S) "; // Start
                } else if (static_cast<int>(goal.x) == x && static_cast<int>(goal.y) == y) {
                    std::cout << "(G) "; // Goal
                } else if (is_path) {
                    std::cout << " *  "; // Planned Path
                } else if (!env.isValid(x, y)) {
                    std::cout << "[#] "; // Obstacle Wall
                } else {
                    std::cout << " .  "; // Free Space
                }
            }
            std::cout << "\n";
        }
        std::cout << "----------------------------------------------------------\n";
    }

    return 0;
}