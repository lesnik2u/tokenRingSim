#pragma once
#include "graphics/Visualizer.h"
#include "core/SimulationManager.h"

class UserInterface {
public:
    UserInterface(SimulationManager& sim, Visualizer& visualizer);
    
    auto render() -> void;
    auto isMouseCaptured() const -> bool;

private:
    SimulationManager& sim;
    Visualizer& visualizer;

    bool showDebugPanel{true};
    bool showNodeInspector{true};
    bool showNetworkController{true};
    // bool showTrafficLog{true};
    int replicationFactor{2}; // UI state for RF
    float messageSpeed{0.5f}; // UI state for speed
    bool mobilityEnabled{true};
    bool ringFormation{true};
    
    int selectedRingIndex{-1}; // Index of the currently selected ring in SimulationManager
    int lastSelectedRingIndex{-1}; // To detect selection changes

    // Inspector State
    // We use sim.getSelectedNodes() instead of selectedNodeId

    auto renderNetworkController() -> void;
    auto renderNodeInspector() -> void;
    auto renderOverlay() -> void;
    auto renderTrafficLog() -> void;
};