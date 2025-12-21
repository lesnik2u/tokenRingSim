#pragma once
#include "Ring.h"
#include <memory>
#include <raylib.h>
#include <string>
#include <unordered_map>
#include <vector>

class Visualizer;

class SimulationManager {
public:
    SimulationManager();

    Ring &addRing(Vector2 center, float radius);
    void removeRing(int index);
    void update(float dt);
    void setVisualizer(Visualizer *vis) { visualizer = vis; }

    void handleInput(const Camera2D &camera);

    void clearSelection();
    void selectNode(int nodeId, bool multiSelect);
    const std::vector<int> &getSelectedNodes() const { return selectedNodes; }
    void onNodeRemoved(int nodeId);

    // Accessors
    std::vector<std::unique_ptr<Ring>> &getRings() { return rings; }
    const std::vector<std::unique_ptr<Ring>> &getRings() const { return rings; }

    // Helpers
    Node *findNodeById(int nodeId) const;

    void setPainting(bool enabled) { isPainting = enabled; }
    bool getPainting() const { return isPainting; }

    bool isBoxSelectionActive() const { return isBoxSelecting; }
    Rectangle getBoxSelectionRect() const {
        float x = std::min(boxStart.x, boxEnd.x);
        float y = std::min(boxStart.y, boxEnd.y);
        float w = std::abs(boxEnd.x - boxStart.x);
        float h = std::abs(boxEnd.y - boxStart.y);
        return {x, y, w, h};
    }

    struct BenchmarkData {
        int nodeCount;
        float fps;
        float minFps;
        float maxFps;
        double physicsMs;
        double renderMs;
    };

    // Benchmarking
    void startBenchmark();
    void stopBenchmark();
    std::string getBenchmarkStatus() const;
    bool isBenchmarkActive() const { return isBenchmarking; }

private:
    std::vector<std::unique_ptr<Ring>> rings;
    std::vector<int> selectedNodes;
    Visualizer *visualizer{nullptr};
    int nextRingId{0};
    bool isPainting{false};
    std::unordered_map<int, Node *> globalNodeMap;

    bool isBoxSelecting{false};
    Vector2 boxStart{0, 0};
    Vector2 boxEnd{0, 0};

    bool isBenchmarking{false};
    float benchTimer{0.0f};
    int benchStage{0};
    std::vector<BenchmarkData> benchResults;

    int benchSamples{0};
    double benchAccFPS{0.0};
    double benchMinFPS{9999.0};
    double benchMaxFPS{0.0};
    double benchAccPhys{0.0};
    double benchAccRender{0.0};

    void updateBenchmarkLogic(float dt);
};