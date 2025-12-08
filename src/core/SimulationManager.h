#pragma once
#include "Ring.h"
#include <vector>
#include <memory>
#include <raylib.h>
#include <string>

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

    auto setPainting(bool enabled) -> void { isPainting = enabled; }
    auto getPainting() const -> bool { return isPainting; }

    // Benchmark System
    struct BenchmarkData {
        int nodeCount;
        float fps;
        float minFps;
        float maxFps;
        double physicsMs;
        double renderMs;
    };
    auto startBenchmark() -> void;
    auto stopBenchmark() -> void;
    auto getBenchmarkStatus() const -> std::string;
    auto isBenchmarkActive() const -> bool { return isBenchmarking; }

private:
    std::vector<std::unique_ptr<Ring>> rings;
    std::vector<int> selectedNodes;
    Visualizer* visualizer{nullptr};
    int nextRingId{0}; // Counter for unique ring IDs
    bool isPainting{false}; // Painting mode flag

    // Benchmark State
    bool isBenchmarking{false};
    float benchTimer{0.0f};
    int benchStage{0};
    std::vector<BenchmarkData> benchResults;
    
    // Accumulators for precision
    int benchSamples{0};
    double benchAccFPS{0.0};
    double benchMinFPS{9999.0};
    double benchMaxFPS{0.0};
    double benchAccPhys{0.0};
    double benchAccRender{0.0};
    
    auto updateBenchmarkLogic(float dt) -> void;
};
