#pragma once
#include "planner/planner_base.hpp"
#include <vector>

class AStarPlanner : PlannerBase{
    public:
        std::vector<Point2D> plan(const Point2D& start, const Point2D& goal, const Environment& env) override;
};

double getHeuristic(int x1, int y1, int x2, int y2);