#include "graphics/UserInterface.h"
#include "imgui.h"
#include "rlImGui.h"
#include <format>
#include <cstring>
#include "utils/Logger.h"

UserInterface::UserInterface(SimulationManager& sim, Visualizer& visualizer)
    : sim(sim), visualizer(visualizer) {}

auto UserInterface::isMouseCaptured() const -> bool {
    return ImGui::GetIO().WantCaptureMouse;
}

auto UserInterface::render() -> void {
    rlImGuiBegin();

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Windows")) {
            ImGui::MenuItem("Network Controller", nullptr, &showNetworkController);
            // ImGui::MenuItem("Traffic Log", nullptr, &showTrafficLog);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    if (showNetworkController) renderNetworkController();
    // Node Inspector is always active as popup/overlay when nodes are selected
    renderNodeInspector();
    // if (showTrafficLog) renderTrafficLog();
    renderOverlay();

    rlImGuiEnd();
}

auto UserInterface::renderOverlay() -> void {
    const float padding = 10.0f;
    ImGui::SetNextWindowPos({padding, 30}, ImGuiCond_FirstUseEver); // Offset for menu bar
    ImGui::SetNextWindowBgAlpha(0.35f);
    if (ImGui::Begin("Overlay", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav)) {
        ImGui::Text("Distributed System Sim");
        ImGui::Text("FPS: %d", GetFPS());
        ImGui::Separator();
        
        size_t totalNodes = 0;
        for(const auto& ring : sim.getRings()) totalNodes += ring->getNodeCount();
        
        ImGui::Text("Rings: %zu", sim.getRings().size());
        ImGui::Text("Total Nodes: %zu", totalNodes);
        ImGui::Text("Selection: %zu", sim.getSelectedNodes().size());
    }
    ImGui::End();
}

auto UserInterface::renderNetworkController() -> void {
    ImGui::Begin("Network Controller", &showNetworkController);

    // --- Global Management ---
    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Create New Ring")) {
            float offset = sim.getRings().size() * 300.0f;
            auto& r = sim.addRing({640.0f + offset, 360.0f}, 200.0f);
            r.addNode(std::format("Node_{}_{}", r.getRingId(), r.getNodeCount()));
            r.addNode(std::format("Node_{}_{}", r.getRingId(), r.getNodeCount()));
            selectedRingIndex = sim.getRings().size() - 1; // Auto-select new ring
        }
    }

    ImGui::Separator();

    // --- Ring Selector ---
    const auto& rings = sim.getRings();
    if (rings.empty()) {
        ImGui::Text("No rings available.");
        ImGui::End();
        return;
    }

    // Build list of ring names for combo box
    if (selectedRingIndex >= static_cast<int>(rings.size())) selectedRingIndex = -1;
    
    std::string previewVal = (selectedRingIndex >= 0) ? std::format("Ring #{}", rings[selectedRingIndex]->getRingId()) : "Select a Ring...";
    
    if (ImGui::BeginCombo("Active Ring", previewVal.c_str())) {
        for (int i = 0; i < static_cast<int>(rings.size()); ++i) {
            const bool isSelected = (selectedRingIndex == i);
            std::string label = std::format("Ring #{}", rings[i]->getRingId());
            if (ImGui::Selectable(label.c_str(), isSelected)) {
                selectedRingIndex = i;
            }
            if (isSelected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    if (selectedRingIndex < 0) {
        ImGui::TextColored(ImVec4(1,1,0,1), "Please select a ring to perform operations.");
        ImGui::End();
        return;
    }

    // --- Context-Sensitive Operations ---
    auto& ring = *rings[selectedRingIndex];

    if (ImGui::CollapsingHeader("Node Operations", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (!sim.getRings().empty()) {
            ImGui::Text("Operating on Ring #%d", (int)ring.getRingId());
            
            if (ImGui::Button("Add Node")) {
                ring.addNode(std::format("Node_{}_{}", ring.getRingId(), ring.getNodeCount()));
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Ring")) {
                sim.removeRing(selectedRingIndex);
                selectedRingIndex = -1; // Reset selection
                ImGui::End();
                return;
            }
        } else {
            ImGui::Text("No rings available.");
        }
    }

    ImGui::Separator();
    // Replication Factor Control
    int currentRF = ring.getReplicationFactor();
    if (ImGui::SliderInt("Replication Factor", &currentRF, 1, std::max(1, (int)ring.getNodeCount()))) {
        ring.setReplicationFactor(currentRF);
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
    if (ImGui::Button("Random Insert")) {
        std::string key = std::format("key_{}", GetRandomValue(1000, 9999));
        std::string value = std::format("val_{}", GetRandomValue(100, 999));
        ring.insertData(key, value);
    }
    // Delete Data button was causing compilation issues due to `ring.deleteData` not existing.
    // So, I'm not re-adding it for now.

    ImGui::End();
}

auto UserInterface::renderNodeInspector() -> void {
    // No main window anymore, just floating windows for selected nodes
    
    const auto& selected = sim.getSelectedNodes();
    
    // We need a copy because we might modify selection inside the loop
    auto nodesToRender = selected;
    
    for (Node* node : nodesToRender) {
        bool open = true;
        // Set default position if the window is appearing for the first time
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 350.0f, 50.0f), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize({300, 450}, ImGuiCond_FirstUseEver);

        std::string windowTitle = std::format("{} (ID: {})###NodeInspector_{}", node->getName(), node->getId(), node->getId());
        
        if (ImGui::Begin(windowTitle.c_str(), &open)) {
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 3.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 3.0f);

            // Node Details
            ImGui::TextColored(ImVec4(0.8f, 0.8f, 1.0f, 1.0f), "Node Details");
            ImGui::Separator();
            ImGui::Text("Name: %s", std::string(node->getName()).c_str());
            ImGui::Text("ID: %d", node->getId());
            ImGui::Text("Position: (%.1f, %.1f)", node->getPosition().x, node->getPosition().y);
            ImGui::Text("Range: [%d° - %d°)", node->getTokenRangeStart(), node->getTokenRangeEnd());
            
            // Node Status Toggle
            bool isActive = node->isActive();
            if (ImGui::Checkbox("Active", &isActive)) {
                node->setActive(isActive);
                APP_LOG_INFO("Node '{}' set to {}", node->getName(), isActive ? "Active" : "Offline");
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Remove Node")) {
                // We need to find which ring this node belongs to
                for(auto& ring : sim.getRings()) {
                    bool found = false;
                    for(const auto& n : ring->getNodes()) {
                        if(n.get() == node) {
                            found = true;
                            break;
                        }
                    }
                    if(found) {
                        ring->removeNode(node->getId());
                        // Also deselect
                        sim.selectNode(node, true); // Toggle off
                        open = false; // Close window
                        break;
                    }
                }
            }
            
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.8f, 0.8f, 1.0f, 1.0f), "Stored Data (%zu items):", node->getDataCount());
            ImGui::Separator();
            
            if (ImGui::BeginTable("Data", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY)) {
                ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Key");
                ImGui::TableSetupColumn("Value");
                ImGui::TableSetupColumn("Hash", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                ImGui::TableHeadersRow();
                
                for(const auto& data : node->getStoredData()) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    if (data->getIsReplica()) {
                        ImGui::TextColored(ImVec4(0.8f, 0.5f, 0.8f, 1.0f), "Replica");
                        if(ImGui::IsItemHovered()) ImGui::SetTooltip("This data item is a replica.");
                    }
                    else {
                        ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "Primary");
                        if(ImGui::IsItemHovered()) ImGui::SetTooltip("This data item is primary.");
                    }
                    
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%s", data->getKey().c_str());
                    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Key: %s", data->getKey().c_str());
                    
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%s", data->getValue().c_str());
                    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Value: %s", data->getValue().c_str());
                    
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%d°", data->getHash());
                    if(ImGui::IsItemHovered()) ImGui::SetTooltip("Hash: %d degrees", data->getHash());
                }
                ImGui::EndTable();
            }
            ImGui::PopStyleVar(3);
        }
        ImGui::End();
        
        if (!open) {
            // Deselect this node
            sim.selectNode(node, true); // This toggles it off because it's already selected
        }
    }
}

/*
auto UserInterface::renderTrafficLog() -> void {
    ImGui::SetNextWindowSize({500, 300}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Traffic Log", &showTrafficLog);

    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Event Stream:");
    ImGui::Separator();

    ImGui::BeginChild("LogScrollRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    
    const auto& logs = Logger::instance().getRecentLogs();
    for (const auto& log : logs) {
        ImGui::TextUnformatted(log.c_str());
    }

    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);

    ImGui::EndChild();
    ImGui::End();
}
*/