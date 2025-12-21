#include "core/Ring.h"
#include "core/SimulationManager.h"
#include "graphics/UserInterface.h"
#include "graphics/Visualizer.h"
#include "imgui.h"
#include "rlImGui.h"
#include "utils/Logger.h"
#include <format>
#include <raylib.h>

int main() {
    APP_LOG_INFO("Application starting");

    const int screenWidth = 1280;
    const int screenHeight = 720;

    // Initialization
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Token Ring Visualization");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    // Systems
    SimulationManager sim;
    Visualizer visualizer;
    UserInterface userInterface(sim, visualizer);

    sim.setVisualizer(&visualizer);

    // Initial setup
    Ring &ring = sim.addRing(Vector2{640, 360}, 250.0f);
    ring.addNode("Node_A");
    ring.addNode("Node_B");
    ring.addNode("Node_C");
    ring.addNode("Node_D");
    ring.setAllNodesMobile(true);
    ring.spawnToken();

    ring.insertData("user_alice", "data_1");
    ring.insertData("user_bob", "data_2");
    ring.insertData("user_charlie", "data_3");
    ring.insertData("user_diana", "data_4");
    ring.insertData("user_eve", "data_5");

    // Main loop
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // Input
        if (!userInterface.isMouseCaptured()) {
            visualizer.handleInput();
            sim.handleInput(visualizer.getCamera());
        }

        // Update
        sim.update(dt);

        // Render
        BeginDrawing();
        ClearBackground(Color{10, 10, 12, 255});

        visualizer.beginCamera();
        for (const std::unique_ptr<Ring> &r : sim.getRings()) {
            visualizer.drawRing(*r, dt);
        }

        if (sim.isBoxSelectionActive()) {
            visualizer.drawSelectionBox(sim.getBoxSelectionRect());
        }

        visualizer.endCamera();

        userInterface.render();

        EndDrawing();
    }

    // Cleanup
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}