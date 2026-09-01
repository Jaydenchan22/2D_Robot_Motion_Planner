#include "planner/rrt_planner.hpp"
#include <cmath>
#include <algorithm>

RRTPlanner::RRTPlanner(double step_size, int max_iterations, double goal_bias)
    : step_size_(step_size), max_iterations_(max_iterations), goal_bias_(goal_bias) {
    std::random_device rd;
    rng_.seed(rd());
}

double RRTPlanner::getDistance(const Point2D& p1, const Point2D& p2) const {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

int RRTPlanner::getNearestNodeIndex(const std::vector<RRTNode>& tree, const Point2D& sample) const {
    int nearest_idx = 0;
    double min_dist = getDistance(tree[0].point, sample);
    
    for (size_t i = 1; i < tree.size(); ++i) {
        double dist = getDistance(tree[i].point, sample);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = static_cast<int>(i);
        }
    }
    return nearest_idx;
}

Point2D RRTPlanner::steer(const Point2D& from, const Point2D& to) const {
    double dist = getDistance(from, to);
    if (dist <= step_size_) {
        return to;
    }
    double angle = std::atan2(to.y - from.y, to.x - from.x);
    return {from.x + step_size_ * std::cos(angle), from.y + step_size_ * std::sin(angle)};
}

bool RRTPlanner::isPathClear(const Point2D& from, const Point2D& to, const Environment& env) const {
    double dist = getDistance(from, to);
    int steps = static_cast<int>(std::ceil(dist / 0.1));
    if (steps == 0) return true;

    double dx = (to.x - from.x) / steps;
    double dy = (to.y - from.y) / steps;
    
    double curr_x = from.x;
    double curr_y = from.y;
    
    for (int i = 0; i <= steps; ++i) {
        if (!env.isValid(static_cast<int>(std::round(curr_x)), static_cast<int>(std::round(curr_y)))) {
            return false;
        }
        curr_x += dx;
        curr_y += dy;
    }
    return true;
}

//Initialize Planner State for animation
void RRTPlanner::init(const Point2D& start, const Point2D& goal, const Environment& env) {
    start_ = start;
    goal_ = goal;
    tree_.clear();
    path_.clear();
    current_iteration_ = 0;

    if (!env.isValid(static_cast<int>(start.x), static_cast<int>(start.y)) ||
        !env.isValid(static_cast<int>(goal.x), static_cast<int>(goal.y))) {
        status_ = RRTStatus::FAILED;
        return;
    }

    tree_.push_back({start_, -1}); // Add start node
    status_ = RRTStatus::SEARCHING;
}

// 2. Perform ONE step of RRT per frame call
RRTStatus RRTPlanner::step(const Environment& env) {
    if (status_ != RRTStatus::SEARCHING) return status_;

    if (current_iteration_ >= max_iterations_) {
        status_ = RRTStatus::FAILED;
        return status_;
    }

    current_iteration_++;

    //Sample point
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    std::uniform_real_distribution<double> x_dist(0.0, static_cast<double>(env.getWidth() - 1.001));
    std::uniform_real_distribution<double> y_dist(0.0, static_cast<double>(env.getHeight() - 1.001));

    if (bias_dist(rng_) < goal_bias_) {
        q_rand_ = goal_;
    } else {
        q_rand_ = {x_dist(rng_), y_dist(rng_)};
    }

    // B. Nearest node
    int nearest_idx = getNearestNodeIndex(tree_, q_rand_);
    Point2D q_near = tree_[nearest_idx].point;

    // C. Steer
    q_new_ = steer(q_near, q_rand_);

    // D. Collision Check
    if (isPathClear(q_near, q_new_, env)) {
        tree_.push_back({q_new_, nearest_idx});
        int new_node_idx = static_cast<int>(tree_.size()) - 1;

        // E. Goal check
        if (getDistance(q_new_, goal_) <= step_size_) {
            if (isPathClear(q_new_, goal_, env)) {
                tree_.push_back({goal_, new_node_idx});

                // Reconstruct Path
                int curr = static_cast<int>(tree_.size()) - 1;
                while (curr != -1) {
                    path_.push_back(tree_[curr].point);
                    curr = tree_[curr].parent_index;
                }
                std::reverse(path_.begin(), path_.end());
                status_ = RRTStatus::FOUND;
            }
        }
    }

    return status_;
}

//Batch plan fallback
std::vector<Point2D> RRTPlanner::plan(const Point2D& start, const Point2D& goal, const Environment& env) {
    init(start, goal, env);
    while (status_ == RRTStatus::SEARCHING) {
        step(env);
    }
    return path_;
}