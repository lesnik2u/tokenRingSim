#pragma once
#include "core/Ring.h"
#include <memory>
#include <raylib.h>
#include <vector>

struct DataTransfer {
    Vector2 fromPos;
    Vector2 toPos;
    float progress;
    std::string dataKey;
    Color color;
    bool isReplication; // To differentiate primary from replica transfers
};

class Visualizer {
private:
    Camera2D camera;
    Vector2 lastMousePos{0, 0};
    bool isPanning{false};
    float animationTime{0.0f};

    // std::vector<Particle> particles;
    std::vector<Vector2> tokenTrail;

    auto drawNodesAdditive(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) -> void;
    auto drawNodesText(const std::vector<std::unique_ptr<Node>>& nodes) -> void;
    auto drawConnections(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) -> void;
    
    auto drawToken(Vector2 from, Vector2 to, float progress) -> void;
    auto drawTokenTrail() -> void;
    auto drawRingGlow(const Ring &ring) -> void;

    // auto spawnParticles(Vector2 position, Color color, int count) -> void;
    // auto updateParticles(float dt) -> void;

    std::vector<DataTransfer> activeTransfers;

    auto drawDataTransfers(float dt) -> void;

public:
    Visualizer();

    auto handleInput() -> void;
    auto drawRing(const Ring &ring, float dt) -> void;
    auto beginCamera() -> void;
    auto endCamera() -> void;
    auto getCamera() const -> const Camera2D & { return camera; }
    auto addTokenTrailPoint(Vector2 point) -> void;
    auto drawDataDistribution(const Ring &ring, Vector2 position) -> void;
    auto startDataTransfer(Vector2 from, Vector2 to, std::string key, bool isReplication = false) -> void;
    auto checkNodeClick(const Ring &ring) -> int; // Returns the ID of the clicked node, or -1 if no node was clicked
    auto drawSelectedNodeHighlight(const Node &node) -> void;

private:
    RenderTexture2D nodeTexture;
    auto bakeNodeTexture() -> void;
};
