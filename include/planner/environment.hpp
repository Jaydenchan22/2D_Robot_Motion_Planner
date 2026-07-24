#pragma once
#include <vector>
#include <iostream>
#include "planner/state.hpp"

class Environment {
    public:

        //initializes a grid of size width x height
        Environment(int width, int height);

        //set an obstacle at (x,y), which means setting it to (1)
        void setObstacle(int x, int y);

         //remove an obstacle at (x,y), which means setting it to (0)
        void removeObstacle(int x, int y);

        //check if the grid is valid
        bool isValid(int x, int y) const;

        //print the map
        void printMap() const;

        int getWidth() const { return width_;}
        int getHeight() const {return height_;}

    private:
        int width_;
        int height_;

        //1D vector to represend the 2D grid
        std::vector<int> grid_;



};