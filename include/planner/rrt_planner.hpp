#pragma once
#include "planner/planner_base.hpp"
#include <vector>
#include <random>

// Represents a node in the tree structure
struct RRTNode {
    Point2D point;
    int parent_index; // Index of the parent node in the tree array (-1 for root)
};

class RRTPlanner : public PlannerBase {
public:
    // Constructor with configurable parameters
    RRTPlanner(double step_size = 0.5, int max_iterations = 5000, double goal_bias = 0.05);

    // Fulfills the PlannerBase contract
    std::vector<Point2D> plan(const Point2D& start, const Point2D& goal, const Environment& env) override;

private:
    double step_size_;      // Distance to grow the tree in each step
    int max_iterations_;    // Maximum random samples before giving up
    double goal_bias_;      // Probability (e.g. 5%) of sampling the goal directly

    //Random Number Generator members
    std::mt19937 rng_;

    // Helper functions
    double getDistance(const Point2D& p1, const Point2D& p2) const;
    int getNearestNodeIndex(const std::vector<RRTNode>& tree, const Point2D& sample) const;
    Point2D steer(const Point2D& from, const Point2D& to) const;
    bool isPathClear(const Point2D& from, const Point2D& to, const Environment& env) const;
};