#include "planner/rrt_planner.hpp"
#include <cmath>
#include <algorithm>

// Constructor: Notice the order matches your new header, and we seed rng_ here!
RRTPlanner::RRTPlanner(double step_size, int max_iterations, double goal_bias)
    : step_size_(step_size), max_iterations_(max_iterations), goal_bias_(goal_bias) {
    
    // Seed the class-member random number generator once when the planner is created
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
        return to; // If it's already close enough, just return the target point
    }
    double angle = std::atan2(to.y - from.y, to.x - from.x);
    return {from.x + step_size_ * std::cos(angle), from.y + step_size_ * std::sin(angle)};
}

bool RRTPlanner::isPathClear(const Point2D& from, const Point2D& to, const Environment& env) const {
    double dist = getDistance(from, to);
    int steps = static_cast<int>(std::ceil(dist / 0.1)); 
    
    // Protect against division by zero if from and to are the exact same point
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

std::vector<Point2D> RRTPlanner::plan(const Point2D& start, const Point2D& goal, const Environment& env) {
    // 1. Boundary / Invalid Start Check
    if (!env.isValid(static_cast<int>(start.x), static_cast<int>(start.y)) ||
        !env.isValid(static_cast<int>(goal.x), static_cast<int>(goal.y))) {
        return {};
    }

    // 2. Initialize Tree
    std::vector<RRTNode> tree;
    tree.push_back({start, -1}); // Root node has no parent (-1)

    // Setup distributions for random point generation
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    std::uniform_real_distribution<double> x_dist(0.0, static_cast<double>(env.getWidth() - 1));
    std::uniform_real_distribution<double> y_dist(0.0, static_cast<double>(env.getHeight() - 1));

    // 3. Main RRT Loop
    for (int iter = 0; iter < max_iterations_; ++iter) {
        
        // --- STEP A: Random Sample ---
        Point2D q_rand;
        if (bias_dist(rng_) < goal_bias_) { // Notice we pass rng_ here
            q_rand = goal;
        } else {
            q_rand = {x_dist(rng_), y_dist(rng_)};
        }

        // --- STEP B: Nearest Neighbor ---
        int nearest_idx = getNearestNodeIndex(tree, q_rand);
        Point2D q_near = tree[nearest_idx].point;

        // --- STEP C: Steer ---
        Point2D q_new = steer(q_near, q_rand);

        // --- STEP D: Collision Check ---
        if (isPathClear(q_near, q_new, env)) {
            tree.push_back({q_new, nearest_idx}); // Add to tree!
            int new_node_idx = static_cast<int>(tree.size()) - 1;

            // --- STEP E: Check if Goal Reached ---
            if (getDistance(q_new, goal) <= step_size_) {
                if (isPathClear(q_new, goal, env)) {
                    
                    // Connect final node to the goal
                    tree.push_back({goal, new_node_idx});
                    
                    // Reconstruct path going backwards
                    std::vector<Point2D> path;
                    int curr = static_cast<int>(tree.size()) - 1;
                    while (curr != -1) {
                        path.push_back(tree[curr].point);
                        curr = tree[curr].parent_index; // Jump to parent
                    }
                    std::reverse(path.begin(), path.end()); // Flip it from Start->Goal
                    
                    return path;
                }
            }
        }
    }

    return {}; // No path found after max_iterations_
}