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
            ImGui::MenuItem("Traffic Log", nullptr, &showTrafficLog);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    if (showNetworkController) renderNetworkController();
    // Node Inspector is always active as popup/overlay when nodes are selected
    renderNodeInspector();
    if (showTrafficLog) renderTrafficLog();
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
            r.addNode("Initial_Node");
            r.addNode("Initial_Node_2");
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
        // Control the FIRST ring for now, or selected nodes?
        // Let's control the LAST created ring as "Active" context or just the first one
        if (!sim.getRings().empty()) {
            auto& ring = *sim.getRings()[selectedRingIndex]; // Operate on selected ring
            
            ImGui::Text("Operating on Ring #%d", (int)ring.getRingId());
            
            if (ImGui::Button("Add Node")) {
                ring.addNode(std::format("Node_{}_{}", ring.getRingId(), ring.getNodeCount()));
            }
            ImGui::SameLine();
            if (ImGui::Button("Remove Ring")) {
                sim.removeRing(selectedRingIndex);
                selectedRingIndex = -1; // Reset selection
                // Note: Loop continues but 'ring' ref might be invalid?
                // We should return immediately to avoid accessing destroyed ring
                ImGui::End();
                return;
            }

            if (ImGui::Button("Remove Node") && ring.getNodeCount() > 2) {
                ring.removeLastNode();
            }
            
            if (ImGui::Button("Reorganize Topology")) {
                ring.sortNodesByPosition();
            }

            ImGui::Checkbox("Enable Node Mobility", &mobilityEnabled);
            
            if (mobilityEnabled) {
                ImGui::Indent();
                ImGui::Checkbox("Ring Formation Force", &ringFormation);
                for(auto& r : sim.getRings()) r->setRingFormation(ringFormation);
                ImGui::Unindent();
            }
            
            // Replication Factor Control
            // Sync UI value with ring's actual value ONLY when selection changes
            if (selectedRingIndex != lastSelectedRingIndex) {
                replicationFactor = ring.getReplicationFactor();
                lastSelectedRingIndex = selectedRingIndex;
            }

            ImGui::SliderInt("Replication Factor", &replicationFactor, 1, std::max(1, (int)ring.getNodeCount()));
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                ring.setReplicationFactor(replicationFactor);
                SimLogInfo("Replication Factor for Ring {} set to {}", ring.getRingId(), replicationFactor);
            }
            
            // Message Speed Control
            if (messageSpeed != ring.getMessageSpeed()) {
                messageSpeed = ring.getMessageSpeed();
            }
            if (ImGui::SliderFloat("Transfer Speed", &messageSpeed, 0.1f, 5.0f)) {
                ring.setMessageSpeed(messageSpeed);
            }
        } else {
            ImGui::Text("No rings available.");
        }
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
        ImGui::SameLine();
        if (ImGui::Button("Delete Data")) {
            if (strlen(keyBuf) > 0) {
                ring.deleteData(std::string(keyBuf));
                keyBuf[0] = '\0';
            }
        }
    }

    ImGui::End();
}

auto UserInterface::renderNodeInspector() -> void {
    // No main window anymore, just floating windows for selected nodes
    
    const auto& selected = sim.getSelectedNodes();
    
    // We need a copy because we might modify selection inside the loop
    auto nodesToRender = selected;
    
    for (Node* node : nodesToRender) {
        bool open = true;
        std::string windowTitle = std::format("{}###Node_{}", node->getName(), node->getId());
        
        // Set default size/pos for new windows
        ImGui::SetNextWindowSize({300, 400}, ImGuiCond_FirstUseEver);
        
        if (ImGui::Begin(windowTitle.c_str(), &open)) {
            ImGui::Text("ID: %d", node->getId());
            ImGui::Text("Range: [%d - %d)", node->getTokenRangeStart(), node->getTokenRangeEnd());
            
            // Node Status Toggle
            bool isActive = node->isActive();
            if (ImGui::Checkbox("Active", &isActive)) {
                node->setActive(isActive);
                SimLogInfo("Node '{}' set to {}", node->getName(), isActive ? "Active" : "Offline");
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Remove Node")) {
                // We need to find which ring this node belongs to
                for(auto& ring : sim.getRings()) {
                    // Checking if node is in this ring is slow but safe
                    // Optimization: Node could store ringId?
                    // For now, search by ID
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
            ImGui::Text("Stored Data (%zu items):", node->getDataCount());
            
            if (ImGui::BeginTable("Data", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
                ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                ImGui::TableSetupColumn("Key");
                ImGui::TableSetupColumn("Hash", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                ImGui::TableHeadersRow();
                
                for(const auto& data : node->getStoredData()) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    if (data->getIsReplica()) ImGui::TextColored(ImVec4(0.8f, 0.5f, 0.8f, 1.0f), "Replica");
                    else ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "Primary");
                    
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%s", data->getKey().c_str());
                    if(ImGui::IsItemHovered()) ImGui::SetTooltip("%s", data->getValue().c_str());
                    
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%d", data->getHash());
                }
                ImGui::EndTable();
            }
        }
        ImGui::End();
        
        if (!open) {
            // Deselect this node
            sim.selectNode(node, true); // This toggles it off because it's already selected
        }
    }
}

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