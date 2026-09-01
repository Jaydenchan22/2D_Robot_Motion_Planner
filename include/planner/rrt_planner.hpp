#pragma once
#include "planner/planner_base.hpp"
#include <vector>
#include <random>

enum class RRTStatus {
    NOT_STARTED,
    SEARCHING,
    FOUND,
    FAILED
};

struct RRTNode {
    Point2D point;
    int parent_index;
};

class RRTPlanner : public PlannerBase {
public:
    RRTPlanner(double step_size = 0.5, int max_iterations = 5000, double goal_bias = 0.05);

    // Batch planning (runs loop to completion)
    std::vector<Point2D> plan(const Point2D& start, const Point2D& goal, const Environment& env) override;

    // Step-by-step animation interface
    void init(const Point2D& start, const Point2D& goal, const Environment& env);
    RRTStatus step(const Environment& env);

    // Getters for Raylib renderer
    const std::vector<RRTNode>& getTree() const { return tree_; }
    const std::vector<Point2D>& getPath() const { return path_; }
    RRTStatus getStatus() const { return status_; }
    Point2D getQRand() const { return q_rand_; }
    Point2D getQNew() const { return q_new_; }

private:
    double step_size_;
    int max_iterations_;
    double goal_bias_;

    std::mt19937 rng_;

    // Animation state storage
    Point2D start_;
    Point2D goal_;
    int current_iteration_ = 0;
    RRTStatus status_ = RRTStatus::NOT_STARTED;

    std::vector<RRTNode> tree_;
    std::vector<Point2D> path_;

    Point2D q_rand_{0, 0};
    Point2D q_new_{0, 0};

    // Helper functions
    double getDistance(const Point2D& p1, const Point2D& p2) const;
    int getNearestNodeIndex(const std::vector<RRTNode>& tree, const Point2D& sample) const;
    Point2D steer(const Point2D& from, const Point2D& to) const;
    bool isPathClear(const Point2D& from, const Point2D& to, const Environment& env) const;
};