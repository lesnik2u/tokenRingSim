#pragma once
#include "core/SimulationManager.h"
#include "graphics/Visualizer.h"
#include <map>
#include <vector>

class UserInterface {
public:
    UserInterface(SimulationManager &sim, Visualizer &visualizer);

    // Main interaction
    void render();
    bool isMouseCaptured() const;

private:
    SimulationManager &sim;
    Visualizer &visualizer;

    // View state
    bool showDebugPanel{true};
    bool showNodeInspector{true};
    bool showNetworkController{true};
    int replicationFactor{2};
    float messageSpeed{0.5f};
    bool mobilityEnabled{true};
    bool ringFormation{true};

    int selectedRingIndex{-1};
    int lastSelectedRingIndex{-1};

    // Cached inspector data
    std::map<int, std::vector<int>> cachedClusters;
    uint64_t lastClusterTopologyVersion = 0;

    // Internal sub-renders
    void renderNetworkController();
    void renderNodeInspector();
    void renderOverlay();
    void renderTrafficLog();

    // Style
    void setupStyle();
};