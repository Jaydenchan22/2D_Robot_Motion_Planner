#include <iostream>
#include <vector>
#include <raylib.h>

#include "planner/state.hpp"
#include "planner/environment.hpp"
#include "planner/astar_planner.hpp"
#include "planner/rrt_planner.hpp"

// Enum to switch active visualization modes
enum class PlannerMode {
    ASTAR,
    RRT
};

int main() {
    //  Setup Map Environment (20x15 grid with walls)
    int grid_width = 20;
    int grid_height = 15;
    Environment env(grid_width, grid_height);

    // for (int y = 2; y <= 12; ++y) env.setObstacle(7, y);  // First wall
    // for (int y = 0; y <= 10; ++y) env.setObstacle(14, y); // Second wall

    Point2D start{2.0, 7.0};
    Point2D goal{18.0, 7.0};

    // Instantiate Both Planners
    AStarPlanner astar_planner;
    RRTPlanner rrt_planner(0.6, 5000, 0.05); // step_size=0.6, max_iter=3000, goal_bias=0.08

    // Initialize both planners
    astar_planner.init(start, goal, env);
    rrt_planner.init(start, goal, env);

    PlannerMode current_mode = PlannerMode::ASTAR;
    bool is_paused = true;

    // Initialize Raylib Window
    int cell_size = 40;
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(env.getWidth() * cell_size, env.getHeight() * cell_size, "2D Motion Planner - Dual Mode (A* & RRT)");
    SetTargetFPS(60);

    // Main Raylib Loop
    while (!WindowShouldClose()) {
        
        //cell size adjust
        cell_size = std::min(GetScreenWidth() / env.getWidth(), GetScreenHeight() / env.getHeight());
        if (cell_size < 1) cell_size = 1;

        //  KEYBOARD CONTROLS 
        if (IsKeyPressed(KEY_ONE)) {
            current_mode = PlannerMode::ASTAR;
            astar_planner.init(start, goal, env); // Reset A* on switch
            is_paused = false;
        }
        if (IsKeyPressed(KEY_TWO)) {
            current_mode = PlannerMode::RRT;
            rrt_planner.init(start, goal, env);   // Reset RRT on switch
            is_paused = false;
        }
        if (IsKeyPressed(KEY_SPACE)) {
            is_paused = !is_paused;
        }
        if (IsKeyPressed(KEY_R)) {
            if (current_mode == PlannerMode::ASTAR) astar_planner.init(start, goal, env);
            else rrt_planner.init(start, goal, env);
            is_paused = true;
        }

        // RESIZE ENVIRONMENT CONTROLS
        bool resize_triggered = false;
        if (IsKeyPressed(KEY_UP)) { grid_height++; resize_triggered = true; }
        if (IsKeyPressed(KEY_DOWN) && grid_height > 5) { grid_height--; resize_triggered = true; }
        if (IsKeyPressed(KEY_RIGHT)) { grid_width++; resize_triggered = true; }
        if (IsKeyPressed(KEY_LEFT) && grid_width > 5) { grid_width--; resize_triggered = true; }

        if (resize_triggered) {
            // Re-create the environment with new dimensions
            env = Environment(grid_width, grid_height);

            // Clamp start and goal points so they don't fall outside the new window
            if (start.x >= grid_width) start.x = grid_width - 1.0;
            if (start.y >= grid_height) start.y = grid_height - 1.0;
            if (goal.x >= grid_width) goal.x = grid_width - 1.0;
            if (goal.y >= grid_height) goal.y = grid_height - 1.0;

            

            // Reset the active planner
            if (current_mode == PlannerMode::ASTAR) astar_planner.init(start, goal, env);
            else rrt_planner.init(start, goal, env);
        }

        //customize obsticles
        // Left Click = Draw Wall, Right Click = Erase Wall
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            
            int grid_x = GetMouseX() / cell_size;
            int grid_y = GetMouseY() / cell_size;

            if (grid_x >= 0 && grid_x < env.getWidth() && grid_y >= 0 && grid_y < env.getHeight()) {
                bool state_changed = false;

                // Move Start Point
                if (IsKeyDown(KEY_S) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    start = {(double)grid_x, (double)grid_y};
                    env.removeObstacle(grid_x, grid_y); // Prevent getting trapped in a wall
                    state_changed = true;
                }
                // Move Goal Point
                else if (IsKeyDown(KEY_E) && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                    goal = {(double)grid_x, (double)grid_y};
                    env.removeObstacle(grid_x, grid_y);
                    state_changed = true;
                }
                // Draw/Erase Walls
                else {
                    if (!(grid_x == (int)start.x && grid_y == (int)start.y) && 
                        !(grid_x == (int)goal.x && grid_y == (int)goal.y)) {
                        
                        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                            env.setObstacle(grid_x, grid_y);
                            state_changed = true;
                        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                            env.removeObstacle(grid_x, grid_y);
                            state_changed = true;
                        }
                    }
                }

                // Restart search if the map or points changed
                if (state_changed) {
                    if (current_mode == PlannerMode::ASTAR) astar_planner.init(start, goal, env);
                    else rrt_planner.init(start, goal, env);
                }
            }
        }

        // STEP SEARCH FORWARD 
        if (!is_paused) {
            if (current_mode == PlannerMode::ASTAR && astar_planner.getStatus() == SearchStatus::SEARCHING) {
                astar_planner.step(env);
            } 
            else if (current_mode == PlannerMode::RRT && rrt_planner.getStatus() == RRTStatus::SEARCHING) {
                // Execute 5 steps per frame for RRT to match animation speed
                for (int i = 0; i < 5; ++i) {
                    if (rrt_planner.getStatus() == RRTStatus::SEARCHING) {
                        rrt_planner.step(env);
                    }
                }
            }
        }

        // RENDER 
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // A. Draw Grid Base and Obstacles
        for (int y = 0; y < env.getHeight(); ++y) {
            for (int x = 0; x < env.getWidth(); ++x) {
                int px = x * cell_size;
                int py = y * cell_size;

                if (!env.isValid(x, y)) {
                    DrawRectangle(px, py, cell_size, cell_size, DARKGRAY); // Wall
                }
                DrawRectangleLines(px, py, cell_size, cell_size, {240, 240, 240, 255});
            }
        }

        // Mode Specific Rendering
        std::vector<Point2D> current_path;

        if (current_mode == PlannerMode::ASTAR) {
            const auto& closed_set = astar_planner.getClosedSet();
            const auto& open_set = astar_planner.getOpenSet();
            current_path = astar_planner.getPath();

            // Render A* Closed/Open sets
            for (int y = 0; y < env.getHeight(); ++y) {
                for (int x = 0; x < env.getWidth(); ++x) {
                    int idx = y * env.getWidth() + x;
                    int px = x * cell_size;
                    int py = y * cell_size;

                    if (env.isValid(x, y)) {
                        if (closed_set[idx]) {
                            DrawRectangle(px, py, cell_size, cell_size, {173, 216, 230, 255}); // Closed Set (Light Blue)
                        } else if (open_set[idx]) {
                            DrawRectangle(px, py, cell_size, cell_size, {255, 239, 150, 255}); // Open Set (Yellow)
                        }
                    }
                    DrawRectangleLines(px, py, cell_size, cell_size, {220, 220, 220, 255});
                }
            }
        } 
        else if (current_mode == PlannerMode::RRT) {
            const auto& tree = rrt_planner.getTree();
            current_path = rrt_planner.getPath();

            // Render RRT Tree Branches
            for (const auto& node : tree) {
                if (node.parent_index != -1) {
                    Point2D parent_pt = tree[node.parent_index].point;

                    Vector2 p1 = {(float)(node.point.x * cell_size + cell_size / 2.0),
                                  (float)(node.point.y * cell_size + cell_size / 2.0)};
                    Vector2 p2 = {(float)(parent_pt.x * cell_size + cell_size / 2.0),
                                  (float)(parent_pt.y * cell_size + cell_size / 2.0)};

                    DrawLineEx(p1, p2, 2.0f, DARKGRAY);
                    DrawCircleV(p1, 3.0f, MAROON);
                }
            }

            // Draw current sample point
            if (rrt_planner.getStatus() == RRTStatus::SEARCHING) {
                Point2D q_rand = rrt_planner.getQRand();
                Vector2 rand_v = {(float)(q_rand.x * cell_size + cell_size / 2.0),
                                  (float)(q_rand.y * cell_size + cell_size / 2.0)};
                DrawCircleV(rand_v, 5.0f, GOLD);
            }
        }

        //Draw Solved Path (Common to both)
        if (current_path.size() > 1) {
            for (size_t i = 0; i < current_path.size() - 1; ++i) {
                Vector2 p1 = {(float)(current_path[i].x * cell_size + cell_size / 2.0),
                              (float)(current_path[i].y * cell_size + cell_size / 2.0)};
                Vector2 p2 = {(float)(current_path[i+1].x * cell_size + cell_size / 2.0),
                              (float)(current_path[i+1].y * cell_size + cell_size / 2.0)};
                DrawLineEx(p1, p2, 6.0f, BLUE);
            }
        }

        // Draw Start and Goal Points
        Vector2 s = {(float)(start.x * cell_size + cell_size / 2.0), (float)(start.y * cell_size + cell_size / 2.0)};
        Vector2 g = {(float)(goal.x * cell_size + cell_size / 2.0), (float)(goal.y * cell_size + cell_size / 2.0)};
        DrawCircleV(s, 12.0f, GREEN);
        DrawCircleV(g, 12.0f, RED);

        // HUD & Status Information
        DrawText("1: A* | 2: RRT | SPACE: Pause | R: Reset", 10, 10, 18, BLACK);
        DrawText("Left Click: Draw Wall | Right Click: Erase", 10, 35, 18, DARKGRAY);
        DrawText("Hold 'S' + Click: Set Start | Hold 'G' + Click: Set Goal", 10, 60, 18, DARKGRAY);
        DrawText("Arrow Keys: Resize Map", 10, 85, 18, DARKGRAY);

        if (current_mode == PlannerMode::ASTAR) {
            DrawText("Active: A*", 10, 115, 18, DARKBLUE);
            if (astar_planner.getStatus() == SearchStatus::FOUND) {
                DrawText("STATUS: PATH FOUND!", 10, 140, 20, DARKGREEN);
            }
        } else {
            DrawText("Active: RRT", 10, 115, 18, DARKBLUE);
            if (rrt_planner.getStatus() == RRTStatus::FOUND) {
                DrawText("STATUS: PATH FOUND!", 10, 140, 20, DARKGREEN);
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}