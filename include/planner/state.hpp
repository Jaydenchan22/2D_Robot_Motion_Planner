#pragma once
#include <iostream>

struct Point2D {
    double x;
    double y;

    void print() const {
        std::cout << "Robot Position: (" << x << ", " << y << ")\n";
    }
};