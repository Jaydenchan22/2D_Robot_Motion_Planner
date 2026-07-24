#include "planner/astar_planner.hpp"
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

//Node structure used for A* search
struct Node{
    int x, y;
    int fCost;

    bool operator > (const Node& other) const{
        return (fCost > other.fCost);
    }
};

//find distance of two point, use for finding h(n)
double getHeuristic(int x1, int y1, int x2, int y2){
    int dy = y1-y2;
    int dx = x1-x2;
    return std::sqrt(dy*dy+dx*dx);
}

std::vector<Point2D> AStarPlanner::plan(const Point2D& start, const Point2D& goal, const Environment& env){
    
    int width = env.getWidth();
    int height = env.getHeight();
    int totalCell = width * height;

    //corvert start and goal point into int
    int startX = static_cast<int>(start.x);
    int startY = static_cast<int>(start.y);
    int goalX = static_cast<int>(goal.x);
    int goalY = static_cast<int>(goal.y);

    if (!(env.isValid(start.x, start.y) && env.isValid(goal.x, goal.y))){
        return {};
    }

    
    //arrays for tracking process
    std::vector<double> gCosts(totalCell, std::numeric_limits<double>::infinity());
    std::vector<int> parentIndices(totalCell, -1);
    std::vector<bool> closedSet(totalCell, false);

    // Open set: Priority queue storing nodes to explore, ordered by lowest fCost
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

    //initialize start point info
    int startIdx = startY*width+startX;
    gCosts[startIdx] = 0;
    int StartDistance = getHeuristic(startX, startY, goalX, goalY);
    openSet.push({startX, startY, StartDistance});

    //movement arrays (idx 0 -> top-left, clockwise)
    const int moveX[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    const int moveY[8] = {1, 1, 1, 0, -1, -1, -1, 0};
    const int moveCost[8] = {14, 10, 14, 10, 14, 10, 14, 10};

    while(!openSet.empty()){
        Node currNode = openSet.top();
        openSet.pop();

        int currIdx = currNode.y*width + currNode.x;

        if (currNode.x == goalX && currNode.y == goalY){
            std::vector<Point2D> path;
            int curr = currIdx;
            while (curr != -1) {
                int py = curr / width;
                int px = curr % width;
                path.push_back({static_cast<double>(px), static_cast<double>(py)});
                curr = parentIndices[curr];
            }
            std::reverse(path.begin(), path.end()); 
            return path;
        }

        if (closedSet[currIdx]) continue;
        closedSet[currIdx] = true;

        for(int i = 0; i < 8; ++i){
            int nx = currNode.x + moveX[i];
            int ny = currNode.y + moveY[i];
            int nIdx = ny * width + nx;

            if (!env.isValid(nx,ny) || closedSet[nIdx]) continue;

            int gCostNew = gCosts[currIdx] + moveCost[i];

            if (gCostNew < gCosts[nIdx]){
                gCosts[nIdx] = gCostNew;
                parentIndices[nIdx] = currIdx;

                int fCost = gCostNew + getHeuristic(nx, ny, goalX, goalY)*10;
                openSet.push({nx, ny, fCost});
            }
        }

    }
    


    return {};
}

