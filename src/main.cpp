#include <raylib.h>
#include "rlImGui.h"
#include "imgui.h"
#include "Ring.h"
#include "Visualizer.h"
#include <format>

int main() {
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
    ring.spawnToken();
    
    Visualizer visualizer;
    bool showDebugPanel = true;
    
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        ring.update(dt);
        
        BeginDrawing();
        ClearBackground(BLACK);
        
        visualizer.drawRing(ring);
        
        rlImGuiBegin();
        if (showDebugPanel) {
            ImGui::Begin("Debug Panel", &showDebugPanel);
            ImGui::Text("FPS: %d", GetFPS());
            ImGui::Separator();
            
            if (ImGui::Button("Add Node")) {
                ring.addNode(std::format("Node_{}", ring.getNodeCount()));
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Node") && ring.getNodeCount() > 2) {
                ring.removeLastNode();
            }
            
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
