#pragma once
#include "Ring.h"
#include <vector>
#include <memory>
#include <raylib.h>

class Visualizer;

class SimulationManager {
public:
    SimulationManager();

    auto addRing(Vector2 center, float radius) -> Ring&;
    auto removeRing(int index) -> void;
    auto update(float dt) -> void;
    auto setVisualizer(Visualizer* vis) -> void { visualizer = vis; }
    
    // Input Handling
    auto handleInput(const Camera2D& camera) -> void;
    
    // Selection
    auto clearSelection() -> void;
    auto selectNode(int nodeId, bool multiSelect) -> void;
    auto getSelectedNodes() const -> const std::vector<int>& { return selectedNodes; }
    auto onNodeRemoved(int nodeId) -> void;

    // Getters
    auto getRings() -> std::vector<std::unique_ptr<Ring>>& { return rings; }
    auto getRings() const -> const std::vector<std::unique_ptr<Ring>>& { return rings; }

    // Helper to find node by ID across all rings
    auto findNodeById(int nodeId) const -> Node*;

private:
    std::vector<std::unique_ptr<Ring>> rings;
    std::vector<int> selectedNodes;
    Visualizer* visualizer{nullptr};
    int nextRingId{0}; // Counter for unique ring IDs

};
