cmake_minimum_required(VERSION 3.22)  

# Project Name and Language
project(MotionPlanner2D VERSION 1.0 LANGUAGES CXX C)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)

message(STATUS "Fetching Raylib...")
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG master
)
FetchContent_MakeAvailable(raylib)

# Tell CMake where your header (.hpp) files live
include_directories(include)

# Automatically find all C++ source files in the src/ folder
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create the executable named "motion_planner.exe"
add_executable(motion_planner ${SOURCES})

target_link_libraries(motion_planner PRIVATE raylib)