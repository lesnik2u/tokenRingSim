#pragma once
#include "core/Ring.h"
#include <memory>
#include <raylib.h>
#include <rlgl.h>
#include <string>
#include <unordered_map>
#include <vector>

class Ring;
class Node;

enum class LODLevel { HIGH, MEDIUM, LOW };

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
    float animationTime = 0.0f;

    float lodThresholdHigh = 0.4f;
    float lodThresholdMedium = 0.15f;

    bool showText = true;
    bool showVelocity = false;
    bool forceHighDetail = false;
    bool animationsEnabled = true;
    float globalScale = 1.0f;
    float animationSpeed = 1.0f;

    // Internal initialization
    void bakeNodeTexture();

    LODLevel cachedLOD = LODLevel::HIGH;

    std::unordered_map<int, Color> cachedColorMap;
    uint64_t lastTopologyVersion = 0;
    uint64_t lastSelectionVersion = 0;

    // Internal logic
    void updateLOD();
    LODLevel getLODLevel() const { return cachedLOD; }

public:
    Visualizer();
    ~Visualizer();

    // LOD Settings
    void setLODThresholds(float high, float medium) {
        lodThresholdHigh = high;
        lodThresholdMedium = medium;
    }
    float getLODThresholdHigh() const { return lodThresholdHigh; }
    float getLODThresholdMedium() const { return lodThresholdMedium; }

    // Toggle Settings
    void setShowText(bool enabled) { showText = enabled; }
    bool getShowText() const { return showText; }

    void setShowVelocity(bool enabled) { showVelocity = enabled; }
    bool getShowVelocity() const { return showVelocity; }

    void setForceHighDetail(bool enabled) { forceHighDetail = enabled; }
    bool getForceHighDetail() const { return forceHighDetail; }

    void setAnimationsEnabled(bool enabled) { animationsEnabled = enabled; }
    bool getAnimationsEnabled() const { return animationsEnabled; }

    void setGlobalScale(float scale) { globalScale = scale; }
    float getGlobalScale() const { return globalScale; }

    void setAnimationSpeed(float speed) { animationSpeed = speed; }
    float getAnimationSpeed() const { return animationSpeed; }

    // Interaction and Drawing
    void handleInput();
    void drawRing(const Ring &ring, float dt);
    void beginCamera();
    void endCamera();
    const Camera2D &getCamera() const { return camera; }
    void drawDataDistribution(const Ring &ring, Vector2 position);
    void startDataTransfer(Vector2 from, Vector2 to, std::string key, bool isReplication = false);
    void drawSelectedNodeHighlight(const Node &node);
    void drawSelectionBox(Rectangle rect);

private:
    // Low-level drawing methods
    void drawNodesAdditive(const std::vector<std::unique_ptr<Node>> &nodes,
                           const std::unordered_map<int, Color> &colorMap,
                           const std::vector<Vector2> &screenPositions,
                           const std::vector<uint8_t> &visibilityCache);
    void drawConnections(const std::vector<std::unique_ptr<Node>> &nodes,
                         const std::unordered_map<int, Color> &colorMap,
                         const std::vector<Vector2> &screenPositions,
                         const std::vector<uint8_t> &visibilityCache);
    void drawNodesText(const std::vector<std::unique_ptr<Node>> &nodes,
                       const std::vector<Vector2> &screenPositions,
                       const std::vector<uint8_t> &visibilityCache);
    void drawVelocityVectors(const std::vector<std::unique_ptr<Node>> &nodes,
                             const std::vector<Vector2> &screenPositions,
                             const std::vector<uint8_t> &visibilityCache);
    void drawDataTransfers(float dt);
    void drawRingGlow(const Ring &ring);
};