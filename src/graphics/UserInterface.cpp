#include "graphics/UserInterface.h"
#include "imgui.h"
#include "rlImGui.h"
#include <format>
#include <cstring>
#include <map>
#include "utils/Logger.h"

UserInterface::UserInterface(SimulationManager& sim, Visualizer& visualizer)
    : sim(sim), visualizer(visualizer) {
    setupStyle();
}

auto UserInterface::isMouseCaptured() const -> bool {
    return ImGui::GetIO().WantCaptureMouse;
}

auto UserInterface::setupStyle() -> void {
    ImGuiStyle& style = ImGui::GetStyle();
    
    style.WindowRounding = 8.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 4.0f;
    style.FramePadding = ImVec2(8, 4);
    style.ItemSpacing = ImVec2(8, 6);
    
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text]                   = ImVec4(0.90f, 0.90f, 0.95f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.10f, 0.10f, 0.12f, 0.95f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.12f, 0.12f, 0.14f, 0.95f);
    colors[ImGuiCol_Border]                 = ImVec4(0.30f, 0.30f, 0.35f, 0.50f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.20f, 0.20f, 0.22f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.25f, 0.25f, 0.28f, 1.00f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.30f, 0.30f, 0.34f, 1.00f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.10f, 0.10f, 0.12f, 0.70f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.14f, 0.14f, 0.16f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.02f, 0.02f, 0.02f, 0.53f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.51f, 0.51f, 0.51f, 1.00f);
    colors[ImGuiCol_CheckMark]              = ImVec4(0.00f, 0.60f, 0.90f, 1.00f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(0.00f, 0.50f, 0.80f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.00f, 0.60f, 0.90f, 1.00f);
    colors[ImGuiCol_Button]                 = ImVec4(0.20f, 0.25f, 0.30f, 1.00f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.25f, 0.35f, 0.45f, 1.00f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.00f, 0.40f, 0.70f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.20f, 0.25f, 0.30f, 1.00f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.25f, 0.35f, 0.45f, 1.00f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.00f, 0.40f, 0.70f, 1.00f);
    colors[ImGuiCol_Separator]              = ImVec4(0.43f, 0.43f, 0.50f, 0.50f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.10f, 0.40f, 0.75f, 0.78f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.10f, 0.40f, 0.75f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(0.26f, 0.59f, 0.98f, 0.25f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_TabHovered]             = ImVec4(0.25f, 0.35f, 0.45f, 1.00f);
    colors[ImGuiCol_TabActive]              = ImVec4(0.20f, 0.25f, 0.30f, 1.00f);
    colors[ImGuiCol_TabUnfocused]           = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_PlotLines]              = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
    colors[ImGuiCol_PlotLinesHovered]       = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
    colors[ImGuiCol_PlotHistogram]          = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
    colors[ImGuiCol_PlotHistogramHovered]   = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
}

auto UserInterface::render() -> void {
    rlImGuiBegin();

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Windows")) {
            ImGui::MenuItem("Network Controller", nullptr, &showNetworkController);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    if (showNetworkController) renderNetworkController();
    renderNodeInspector();
    renderOverlay();

    rlImGuiEnd();
}

auto UserInterface::renderOverlay() -> void {
    const float padding = 10.0f;
    ImGui::SetNextWindowPos({padding, 30}, ImGuiCond_FirstUseEver); 
    ImGui::SetNextWindowBgAlpha(0.60f); 
    if (ImGui::Begin("Stats Overlay", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav)) {
        ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "Distributed Sim");
        ImGui::Separator();
        ImGui::Text("FPS: %d", GetFPS());
        
        size_t totalNodes = 0;
        for(const auto& ring : sim.getRings()) totalNodes += ring->getNodeCount();
        
        ImGui::Text("Nodes: %zu", totalNodes);
        ImGui::Text("Selected: %zu", sim.getSelectedNodes().size());
    }
    ImGui::End();
}

auto UserInterface::renderNetworkController() -> void {
    ImGui::SetNextWindowSize({400, 500}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Network Controller", &showNetworkController);

    if (sim.getRings().empty()) {
         sim.addRing({640.0f, 360.0f}, 200.0f);
    }
    auto& worldRing = *sim.getRings().front();

    if (ImGui::BeginTabBar("ControllerTabs")) {
        
        // --- Tab 1: General ---
        if (ImGui::BeginTabItem("General")) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "Simulation Control");
            ImGui::Separator();
            
            static int spawnAmount = 50;
            ImGui::SliderInt("Spawn Amount", &spawnAmount, 1, 1000);
            
            if (ImGui::Button("Spawn Nodes", ImVec2(-1, 0))) { 
                for(int i=0; i<spawnAmount; ++i) {
                    worldRing.addNode(std::format("Node_{}", worldRing.getNodeCount()));
                    Node& n = worldRing[worldRing.getNodeCount()-1];
                    n.setPosition({(float)GetRandomValue(100, 1180), (float)GetRandomValue(100, 620)});
                }
            }
            
            ImGui::Spacing();
            
            bool paintMode = sim.getPainting();
            if (ImGui::Checkbox("Paint Mode (Hold Click)", &paintMode)) {
                sim.setPainting(paintMode);
            }
            
            if (ImGui::Button("Clear World", ImVec2(-1, 0))) {
                while(worldRing.getNodeCount() > 0) {
                    worldRing.removeLastNode();
                }
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Mobility:");
            
            if (ImGui::Checkbox("Enable Physics", &mobilityEnabled)) {
                worldRing.setAllNodesMobile(mobilityEnabled);
            }
            
            if (mobilityEnabled) {
                if (ImGui::Checkbox("Emergent Forces", &ringFormation)) {
                    worldRing.setRingFormation(ringFormation);
                }
            }
            
            ImGui::EndTabItem();
        }

        // --- Tab 2: Physics ---
        if (ImGui::BeginTabItem("Physics")) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "Force Parameters");
            ImGui::Separator();

            // Max Cluster Size
            int currentMaxCluster = worldRing.getMaxClusterSize();
            if (ImGui::SliderInt("Max Ring Size", &currentMaxCluster, 5, 100)) {
                worldRing.setMaxClusterSize(currentMaxCluster);
            }
            if(ImGui::IsItemHovered()) ImGui::SetTooltip("Maximum nodes allowed in a single emergent ring before it splits.");

            ImGui::Spacing();

            ImGui::SliderFloat("Search Radius", &worldRing.physics.searchRadius, 50.0f, 300.0f);
            ImGui::SliderFloat("Ideal Spacing", &worldRing.physics.idealDist, 20.0f, 150.0f);
            
            ImGui::Spacing();
            ImGui::Text("Strengths:");
            ImGui::SliderFloat("Attraction", &worldRing.physics.chainAttractStrength, 0.0f, 50.0f);
            ImGui::SliderFloat("Repulsion", &worldRing.physics.repulsionStrength, 0.0f, 200.0f);
            ImGui::SliderFloat("Vortex (Curl)", &worldRing.physics.vortexStrength, 0.0f, 20.0f);
            ImGui::SliderFloat("Gravity (Cluster)", &worldRing.physics.boundaryStrength, 0.0f, 10.0f);
            ImGui::SliderFloat("Split Force", &worldRing.physics.splitStrength, 0.0f, 300.0f);
            
            ImGui::Spacing();
            ImGui::SliderFloat("Friction", &worldRing.physics.friction, 0.80f, 0.99f);
            ImGui::SliderFloat("Max Speed", &worldRing.physics.maxSpeed, 100.0f, 1000.0f);

            ImGui::EndTabItem();
        }

        // --- Tab 3: Inspection ---
        if (ImGui::BeginTabItem("Inspector")) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), "Emergent Rings");
            ImGui::Separator();

            std::map<int, std::vector<int>> clusters;
            for(const auto& node : worldRing.getNodes()) {
                int cid = node->getClusterId();
                if(cid != -1) clusters[cid].push_back(node->getId());
            }
            
            if (clusters.empty()) {
                ImGui::TextDisabled("No rings formed.");
            } else {
                ImGui::BeginChild("ClusterList");
                for(auto& [cid, members] : clusters) {
                    if(members.size() < 3) continue;
                    
                    std::string label = std::format("Ring #{} ({} nodes)", cid, members.size());
                    if (ImGui::Selectable(label.c_str())) {
                        sim.clearSelection();
                        for(int nid : members) sim.selectNode(nid, true);
                    }
                }
                ImGui::EndChild();
            }
            ImGui::EndTabItem();
        }
        
        // --- Tab 4: Data ---
        if (ImGui::BeginTabItem("Data")) {
            ImGui::Spacing();
            static char keyBuf[64] = "";
            static char valueBuf[128] = "";

            ImGui::InputText("Key", keyBuf, sizeof(keyBuf));
            ImGui::InputText("Value", valueBuf, sizeof(valueBuf));

            if (ImGui::Button("Insert Data", ImVec2(-1, 0))) {
                if (strlen(keyBuf) > 0) {
                    worldRing.insertData(std::string(keyBuf), std::string(valueBuf));
                    keyBuf[0] = '\0';
                    valueBuf[0] = '\0';
                }
            }
            
            if (ImGui::Button("Random Insert", ImVec2(-1, 0))) {
                std::string key = std::format("key_{}", GetRandomValue(1000, 9999));
                std::string value = std::format("val_{}", GetRandomValue(100, 999));
                worldRing.insertData(key, value);
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
}

auto UserInterface::renderNodeInspector() -> void {
    const auto& selectedNodeIds = sim.getSelectedNodes();
    if (selectedNodeIds.empty()) return;

    if (selectedNodeIds.size() > 1) {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 350.0f, 50.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({300, 450}, ImGuiCond_FirstUseEver);
        
        if (ImGui::Begin("Multi-Node Inspector", nullptr)) {
            ImGui::TextColored(ImVec4(0.0f, 0.8f, 1.0f, 1.0f), "Selection: %zu Nodes", selectedNodeIds.size());
            ImGui::Separator();
            
            if (ImGui::Button("Deselect All")) {
                sim.clearSelection();
            }
            ImGui::SameLine();
            if (ImGui::Button("Delete Selection")) {
                std::vector<int> ids = selectedNodeIds;
                if (!sim.getRings().empty()) {
                    auto& ring = *sim.getRings().front();
                    for(int id : ids) ring.removeNode(id);
                }
                sim.clearSelection();
            }

            ImGui::Separator();
            ImGui::BeginChild("NodeList");
            for (int nodeId : selectedNodeIds) {
                Node* node = sim.findNodeById(nodeId);
                if (node) {
                    ImGui::Text("Node %d: %s (Cluster %d)", nodeId, std::string(node->getName()).c_str(), node->getClusterId());
                }
            }
            ImGui::EndChild();
        }
        ImGui::End();
        return;
    }

    int nodeId = selectedNodeIds[0];
    Node* node = sim.findNodeById(nodeId);
    if (!node) return; 
    
    bool open = true;
    bool nodeRemoved = false;
    
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 350.0f, 50.0f), ImGuiCond_FirstUseEver); 
    ImGui::SetNextWindowSize({300, 450}, ImGuiCond_FirstUseEver);

    std::string windowTitle = std::format("{} (ID: {})###NodeInspector_{}", node->getName(), node->getId(), node->getId());
    
    if (ImGui::Begin(windowTitle.c_str(), &open)) {
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 1.0f, 1.0f), "Node Details");
        ImGui::Separator();
        ImGui::Text("Name: %s", std::string(node->getName()).c_str());
        ImGui::Text("ID: %d", node->getId());
        ImGui::Text("Ring Cluster: %d", node->getClusterId());
        ImGui::Text("Position: (%.1f, %.1f)", node->getPosition().x, node->getPosition().y);
        
        bool isActive = node->isActive();
        if (ImGui::Checkbox("Active", &isActive)) {
            node->setActive(isActive);
        }
        
        ImGui::SameLine();
        if (ImGui::Button("Remove Node")) {
            if (!sim.getRings().empty()) {
                auto& ring = *sim.getRings().front();
                ring.removeNode(nodeId);
                nodeRemoved = true;
                open = false;
            }
        }
        
        if (!nodeRemoved) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.8f, 0.8f, 1.0f, 1.0f), "Stored Data (%zu items):", node->getDataCount());
            ImGui::Separator();
            
            if (ImGui::BeginTable("Data", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY)) {
                ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Key");
                ImGui::TableSetupColumn("Value");
                ImGui::TableHeadersRow();
                
                for(const auto& data : node->getStoredData()) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", data->getIsReplica() ? "Replica" : "Primary");
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%s", data->getKey().c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%s", data->getValue().c_str());
                }
                ImGui::EndTable();
            }
        }
    }
    ImGui::End();
    
    if (!open && !nodeRemoved) {
        sim.selectNode(nodeId, true);
    }
}