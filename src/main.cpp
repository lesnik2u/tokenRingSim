#include "utils/Logger.h"
#include "core/Ring.h"
#include "graphics/Visualizer.h"
#include "imgui.h"
#include "rlImGui.h"
#include <format>
#include <raylib.h>

int main() {
    LOG_INFO("Application starting");

    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Token Ring Visualization");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    Ring ring(Vector2{640, 360}, 250.0f);
    ring.addNode("Node_A");
    ring.addNode("Node_B");
    ring.addNode("Node_C");
    ring.addNode("Node_D");
    ring.assignTokenRanges(); // ADD THIS
    ring.spawnToken();
    static bool mobilityEnabled = false;
    static bool ringFormation = true;

    Visualizer visualizer;
    ring.setVisualizer(&visualizer);

    ring.insertData("user_alice", "data_1");
    ring.insertData("user_bob", "data_2");
    ring.insertData("user_charlie", "data_3");
    ring.insertData("user_diana", "data_4");
    ring.insertData("user_eve", "data_5");

    bool showDebugPanel = true;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        static float reorganizeTimer = 0.0f;
        reorganizeTimer += dt;

        // Handle camera input (before ImGui)
        if (!ImGui::GetIO().WantCaptureMouse) {
            visualizer.handleInput();

            Vector2 mousePos = GetMousePosition();
            bool mousePressed = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
            ring.handleNodeDragging(mousePos, mousePressed, visualizer.getCamera());
        }

        ring.update(dt);
        ring.updateNodeMovement(dt, Vector2{1280, 720});

        // Apply ring formation forces when mobile
        ring.applyRingFormationForces();

        // Reorganize periodically when nodes are mobile
        if (reorganizeTimer > 0.5f) {
            ring.reorganizeFromPositions();
            reorganizeTimer = 0.0f;
        }
        BeginDrawing();
        ClearBackground(RAYWHITE);

        visualizer.beginCamera();
        visualizer.drawRing(ring, dt); // Pass dt for animations
        visualizer.endCamera();

        rlImGuiBegin();
        if (showDebugPanel) {
            ImGui::Begin("Debug Panel", &showDebugPanel);
            ImGui::Text("FPS: %d", GetFPS());
            ImGui::Separator();

            if (ImGui::Button("Add Node")) {
                ring += std::format("Node_{}", ring.getNodeCount());
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Node") && ring.getNodeCount() > 2) {
                ring -= ring[ring.getNodeCount() - 1].getName();
            }

            if (ImGui::Button("Scale Up")) {
                ring *= 1.1f;
            }
            ImGui::SameLine();
            if (ImGui::Button("Scale Down")) {
                ring *= 0.9f;
            }

            ImGui::Separator();
            if (ImGui::Checkbox("Enable Node Mobility", &mobilityEnabled)) {
                ring.setAllNodesMobile(mobilityEnabled);
            }

            ImGui::Separator();
            ImGui::Text("Data Management:");

            static char keyBuf[64] = "";
            static char valueBuf[128] = "";

            ImGui::InputText("Key", keyBuf, sizeof(keyBuf));
            ImGui::InputText("Value", valueBuf, sizeof(valueBuf));

            if (ImGui::Button("Insert Data")) {
                if (strlen(keyBuf) > 0) {
                    ring.insertData(std::string(keyBuf), std::string(valueBuf));
                    keyBuf[0] = '\0';
                    valueBuf[0] = '\0';
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Random Data")) {
                for (int i = 0; i < 5; ++i) {
                    std::string key = std::format("key_{}", GetRandomValue(1000, 9999));
                    std::string value = std::format("value_{}", GetRandomValue(100, 999));
                    ring.insertData(key, value);
                }
            }

            ImGui::Separator();
            auto dist = ring.getDataDistribution();
            ImGui::Text("Data Distribution:");
            for (size_t i = 0; i < dist.size(); ++i) {
                ImGui::Text("  Node %zu: %d items", i, dist[i]);
            }
            ImGui::Checkbox("Ring Formation", &ringFormation);
            ring.setRingFormation(ringFormation);

            ImGui::Text("Controls:");
            ImGui::BulletText("Left Mouse: Drag nodes");
            ImGui::BulletText("Right Mouse: Pan camera");
            ImGui::BulletText("Mouse Wheel: Zoom");
            ImGui::BulletText("R: Reset camera");

            ImGui::Separator();
            ImGui::Text("Nodes: %zu", ring.getNodeCount());

            ImGui::End();
        }
        rlImGuiEnd();

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
