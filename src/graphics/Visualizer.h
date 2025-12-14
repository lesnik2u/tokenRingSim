#pragma once
#include <raylib.h>
#include <rlgl.h>
#include <vector>
#include <memory>
#include "core/Ring.h"

class Ring;
class Node;

enum class LODLevel {
    HIGH,   // Full detail (Text, Glow, Satellites)
    MEDIUM, // Simplified (No Text, No Satellites)
    LOW     // Minimal (Dots and Thin Lines)
};

class Visualizer {
private:
    Camera2D camera;
    RenderTexture2D nodeTexture;
    bool isPanning = false;
    Vector2 lastMousePos = {0, 0};
    
    struct DataTransfer {
        Vector2 fromPos;
        Vector2 toPos;
        float progress;
        std::string dataKey;
        bool isReplication;
        Color color;
    };
    std::vector<DataTransfer> activeTransfers;
    std::vector<Vector2> tokenTrail;
    float animationTime = 0.0f;

    float lodThresholdHigh = 0.4f;
    float lodThresholdMedium = 0.15f;

    // Visual Settings
    bool showText = true;
    bool showVelocity = false;
    bool forceHighDetail = false;
    bool animationsEnabled = true; // Toggle for pulsing/rotating/orbiting
    float globalScale = 1.0f;
    float animationSpeed = 1.0f;

    void bakeNodeTexture();
    
    // Batching Helpers
    auto getLODLevel() const -> LODLevel {
        if (forceHighDetail) return LODLevel::HIGH;
        if (camera.zoom > lodThresholdHigh) return LODLevel::HIGH;
        if (camera.zoom > lodThresholdMedium) return LODLevel::MEDIUM;
        return LODLevel::LOW;
    }

public:
    Visualizer();
    ~Visualizer();

    auto setLODThresholds(float high, float medium) -> void {
        lodThresholdHigh = high;
        lodThresholdMedium = medium;
    }
    auto getLODThresholdHigh() const -> float { return lodThresholdHigh; }
    auto getLODThresholdMedium() const -> float { return lodThresholdMedium; }

    auto setShowText(bool enabled) -> void { showText = enabled; }
    auto getShowText() const -> bool { return showText; }

    auto setShowVelocity(bool enabled) -> void { showVelocity = enabled; }
    auto getShowVelocity() const -> bool { return showVelocity; }

    auto setForceHighDetail(bool enabled) -> void { forceHighDetail = enabled; }
    auto getForceHighDetail() const -> bool { return forceHighDetail; }

    auto setAnimationsEnabled(bool enabled) -> void { animationsEnabled = enabled; }
    auto getAnimationsEnabled() const -> bool { return animationsEnabled; }

    auto setGlobalScale(float scale) -> void { globalScale = scale; }
    auto getGlobalScale() const -> float { return globalScale; }

    auto setAnimationSpeed(float speed) -> void { animationSpeed = speed; }
    auto getAnimationSpeed() const -> float { return animationSpeed; }

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
    // Helper drawing functions
    auto drawNodesAdditive(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) -> void;
    auto drawConnections(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) -> void;
    auto drawNodesText(const std::vector<std::unique_ptr<Node>>& nodes) -> void;
    auto drawVelocityVectors(const std::vector<std::unique_ptr<Node>>& nodes) -> void;
    auto drawToken(Vector2 from, Vector2 to, float progress) -> void;
    auto drawDataTransfers(float dt) -> void;
    auto drawRingGlow(const Ring &ring) -> void;
    auto drawTokenTrail() -> void;
};
