#include "planner/environment.hpp"

Environment::Environment(int width, int height) 
    : width_(width), height_(height), grid_(width * height, 0) {}

void Environment::setObstacle(int x, int y) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        grid_[y * width_ + x] = 1;
    }
}

void Environment::removeObstacle(int x, int y) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        grid_[y * width_ + x] = 0;
    }
}

bool Environment::isValid(int x, int y) const {
    // 1. Check boundary limits
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }
    // 2. Check collision (must be 0 / free space)
    return grid_[y * width_ + x] == 0;
}

void Environment::printMap() const {
    std::cout << "\n--- Environment Map ---\n";
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y * width_ + x] == 1) {
                std::cout << "[#] "; // Obstacle
            } else {
                std::cout << " .  "; // Free space
            }
        }
        std::cout << "\n";
    }
    std::cout << "-----------------------\n";
}