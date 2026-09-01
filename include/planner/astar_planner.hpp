#pragma once
#include "planner/planner_base.hpp"
#include <vector>
#include <queue>
#include <limits>

enum class SearchStatus {
    NOT_STARTED,
    SEARCHING,
    FOUND,
    NO_PATH
};

struct Node {
    int x, y;
    double f_cost;
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

class AStarPlanner : public PlannerBase {
public:
    // Batch planning (runs search to completion immediately)
    std::vector<Point2D> plan(const Point2D& start, 
                              const Point2D& goal, 
                              const Environment& env) override;

    // Step-by-step planning interface for real-time animation
    void init(const Point2D& start, const Point2D& goal, const Environment& env);
    SearchStatus step(const Environment& env);

    // Getters for visualization rendering
    const std::vector<bool>& getClosedSet() const { return closed_set_; }
    const std::vector<bool>& getOpenSet() const { return in_open_set_; }
    const std::vector<Point2D>& getPath() const { return path_; }
    SearchStatus getStatus() const { return status_; }

private:
    int width_ = 0;
    int height_ = 0;
    int goal_x_ = 0;
    int goal_y_ = 0;
    
    SearchStatus status_ = SearchStatus::NOT_STARTED;

    std::vector<double> g_costs_;
    std::vector<int> parent_indices_;
    std::vector<bool> closed_set_;
    std::vector<bool> in_open_set_;
    std::vector<Point2D> path_;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set_;

    double getHeuristic(int x1, int y1, int x2, int y2) const;
};