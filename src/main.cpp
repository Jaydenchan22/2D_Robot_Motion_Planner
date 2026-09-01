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
    // 1. Setup Map Environment (20x15 grid with walls)
    Environment env(20, 15);
    for (int y = 2; y <= 12; ++y) env.setObstacle(7, y);  // First wall
    for (int y = 0; y <= 10; ++y) env.setObstacle(14, y); // Second wall

    Point2D start{2.0, 7.0};
    Point2D goal{18.0, 7.0};

    // 2. Instantiate Both Planners
    AStarPlanner astar_planner;
    RRTPlanner rrt_planner(0.6, 3000, 0.08); // step_size=0.6, max_iter=3000, goal_bias=0.08

    // Initialize both planners
    astar_planner.init(start, goal, env);
    rrt_planner.init(start, goal, env);

    PlannerMode current_mode = PlannerMode::ASTAR;
    bool is_paused = false;

    // 3. Initialize Raylib Window
    int cell_size = 40;
    InitWindow(env.getWidth() * cell_size, env.getHeight() * cell_size, "2D Motion Planner - Dual Mode (A* & RRT)");
    SetTargetFPS(60);

    // 4. Main Raylib Loop
    while (!WindowShouldClose()) {
        // --- KEYBOARD CONTROLS ---
        if (IsKeyPressed(KEY_ONE)) {
            current_mode = PlannerMode::ASTAR;
            astar_planner.init(start, goal, env); // Reset A* on switch
        }
        if (IsKeyPressed(KEY_TWO)) {
            current_mode = PlannerMode::RRT;
            rrt_planner.init(start, goal, env);   // Reset RRT on switch
        }
        if (IsKeyPressed(KEY_SPACE)) {
            is_paused = !is_paused;
        }
        if (IsKeyPressed(KEY_R)) {
            if (current_mode == PlannerMode::ASTAR) astar_planner.init(start, goal, env);
            else rrt_planner.init(start, goal, env);
        }

        // --- STEP SEARCH FORWARD ---
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

        // --- RENDER ---
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

        // B. Mode Specific Rendering
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

        // C. Draw Solved Path (Common to both)
        if (current_path.size() > 1) {
            for (size_t i = 0; i < current_path.size() - 1; ++i) {
                Vector2 p1 = {(float)(current_path[i].x * cell_size + cell_size / 2.0),
                              (float)(current_path[i].y * cell_size + cell_size / 2.0)};
                Vector2 p2 = {(float)(current_path[i+1].x * cell_size + cell_size / 2.0),
                              (float)(current_path[i+1].y * cell_size + cell_size / 2.0)};
                DrawLineEx(p1, p2, 6.0f, BLUE);
            }
        }

        // D. Draw Start and Goal Points
        Vector2 s = {(float)(start.x * cell_size + cell_size / 2.0), (float)(start.y * cell_size + cell_size / 2.0)};
        Vector2 g = {(float)(goal.x * cell_size + cell_size / 2.0), (float)(goal.y * cell_size + cell_size / 2.0)};
        DrawCircleV(s, 12.0f, GREEN);
        DrawCircleV(g, 12.0f, RED);

        // E. HUD & Status Information
        DrawText("1: A* Mode  |  2: RRT Mode  |  SPACE: Pause  |  R: Reset", 10, 10, 18, BLACK);

        if (current_mode == PlannerMode::ASTAR) {
            DrawText("Active Planner: A* (Grid Search)", 10, 35, 18, DARKBLUE);
            if (astar_planner.getStatus() == SearchStatus::FOUND) {
                DrawText("STATUS: PATH FOUND!", 10, 60, 20, DARKGREEN);
            }
        } else {
            DrawText("Active Planner: RRT (Sampling Tree)", 10, 35, 18, DARKBLUE);
            if (rrt_planner.getStatus() == RRTStatus::FOUND) {
                DrawText("STATUS: PATH FOUND!", 10, 60, 20, DARKGREEN);
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}