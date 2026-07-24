#pragma once
#include <vector>
#include "planner/state.hpp"
#include "planner/environment.hpp"

class PlannerBase {
public:
    virtual ~PlannerBase() = default;

    // Pure virtual function: Every planner MUST implement a plan() method 
    // that takes a start, a goal, and the environment, and returns a path of points.
    virtual std::vector<Point2D> plan(const Point2D& start, const Point2D& goal, const Environment& env) = 0;
};