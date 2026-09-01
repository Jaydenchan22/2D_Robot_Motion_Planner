#include "planner/astar_planner.hpp"
#include <cmath>
#include <algorithm>

double AStarPlanner::getHeuristic(int x1, int y1, int x2, int y2) const {
    return std::hypot(x1 - x2, y1 - y2);
}

void AStarPlanner::init(const Point2D& start, const Point2D& goal, const Environment& env) {
    width_ = env.getWidth();
    height_ = env.getHeight();
    int total_cells = width_ * height_;

    int start_x = static_cast<int>(start.x);
    int start_y = static_cast<int>(start.y);
    goal_x_ = static_cast<int>(goal.x);
    goal_y_ = static_cast<int>(goal.y);

    path_.clear();
    while (!open_set_.empty()) open_set_.pop();

    if (!env.isValid(start_x, start_y) || !env.isValid(goal_x_, goal_y_)) {
        status_ = SearchStatus::NO_PATH;
        return;
    }

    g_costs_.assign(total_cells, std::numeric_limits<double>::infinity());
    parent_indices_.assign(total_cells, -1);
    closed_set_.assign(total_cells, false);
    in_open_set_.assign(total_cells, false);

    int start_idx = start_y * width_ + start_x;
    g_costs_[start_idx] = 0.0;
    
    double start_h = getHeuristic(start_x, start_y, goal_x_, goal_y_);
    open_set_.push({start_x, start_y, start_h});
    in_open_set_[start_idx] = true;

    status_ = SearchStatus::SEARCHING;
}

SearchStatus AStarPlanner::step(const Environment& env) {
    if (status_ != SearchStatus::SEARCHING) return status_;

    if (open_set_.empty()) {
        status_ = SearchStatus::NO_PATH;
        return status_;
    }

    Node current = open_set_.top();
    open_set_.pop();

    int curr_idx = current.y * width_ + current.x;
    in_open_set_[curr_idx] = false;

    if (closed_set_[curr_idx]) return status_;
    closed_set_[curr_idx] = true;

    // Check if goal reached
    if (current.x == goal_x_ && current.y == goal_y_) {
        int curr = curr_idx;
        while (curr != -1) {
            int py = curr / width_;
            int px = curr % width_;
            path_.push_back({static_cast<double>(px), static_cast<double>(py)});
            curr = parent_indices_[curr];
        }
        std::reverse(path_.begin(), path_.end());
        status_ = SearchStatus::FOUND;
        return status_;
    }

    // 8-neighbor exploration
    const int dx[8] = {-1,  1,  0,  0, -1, -1,  1,  1};
    const int dy[8] = { 0,  0, -1,  1, -1,  1, -1,  1};
    const double move_cost[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    for (int i = 0; i < 8; ++i) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        int neighbor_idx = ny * width_ + nx;

        if (!env.isValid(nx, ny) || closed_set_[neighbor_idx]) continue;

        double new_g = g_costs_[curr_idx] + move_cost[i];

        if (new_g < g_costs_[neighbor_idx]) {
            g_costs_[neighbor_idx] = new_g;
            parent_indices_[neighbor_idx] = curr_idx;

            double f_cost = new_g + getHeuristic(nx, ny, goal_x_, goal_y_);
            open_set_.push({nx, ny, f_cost});
            in_open_set_[neighbor_idx] = true;
        }
    }

    return status_;
}

std::vector<Point2D> AStarPlanner::plan(const Point2D& start, const Point2D& goal, const Environment& env) {
    init(start, goal, env);
    while (status_ == SearchStatus::SEARCHING) {
        step(env);
    }
    return path_;
}