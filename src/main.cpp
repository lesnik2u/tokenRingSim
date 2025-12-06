#include "utils/Logger.h"
#include "core/Ring.h"
#include "core/SimulationManager.h"
#include "graphics/Visualizer.h"
#include "graphics/UserInterface.h"
#include "imgui.h"
#include "rlImGui.h"
#include <format>
#include <raylib.h>

int main() {
    APP_LOG_INFO("Application starting");

    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Token Ring Visualization");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    SimulationManager sim;
    Visualizer visualizer;
    UserInterface userInterface(sim, visualizer);

    sim.setVisualizer(&visualizer);

    auto& ring = sim.addRing(Vector2{640, 360}, 250.0f);
    ring.addNode("Node_A");
    ring.addNode("Node_B");
    ring.addNode("Node_C");
    ring.addNode("Node_D");
    ring.spawnToken(); // This must be called after nodes are added.

    ring.insertData("user_alice", "data_1");
    ring.insertData("user_bob", "data_2");
    ring.insertData("user_charlie", "data_3");
    ring.insertData("user_diana", "data_4");
    ring.insertData("user_eve", "data_5");

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // Handle input only if UI is not capturing mouse
        if (!userInterface.isMouseCaptured()) {
            visualizer.handleInput();
            sim.handleInput(visualizer.getCamera()); // Pass camera for world to screen conversion
        }
        
        sim.update(dt); // Update simulation logic

        BeginDrawing();
        ClearBackground(RAYWHITE);

        visualizer.beginCamera();
        for (const auto& r : sim.getRings()) {
            visualizer.drawRing(*r, dt);
        }
        visualizer.endCamera();

        userInterface.render(); // Render UI on top

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
