cmake_minimum_required(VERSION 3.15)  

# Project Name and Language
project(MotionPlanner2D VERSION 1.0 LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where your header (.hpp) files live
include_directories(include)

# Automatically find all C++ source files in the src/ folder
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create the executable named "motion_planner.exe"
add_executable(motion_planner ${SOURCES})