#pragma once
#include "core/Node.h"
#include "core/Token.h"
#include <memory>
#include <raylib.h>
#include <vector>
#include "graphics/Visualizer.h"

class Visualizer;

/**
 * @brief Manages the Distributed Hash Table (DHT) ring.
 * 
 * Implements a Consistent Hashing strategy where:
 * - The ring represents a 0-360 degree key space.
 * - Nodes are assigned a position on the ring (angle).
 * - Data items are hashed to an angle and stored on the first node
 *   encountered moving clockwise (or in the node's assigned range).
 * - Supports dynamic node addition/removal with data repartitioning.
 */
class Ring {
private:
    std::vector<std::unique_ptr<Node>> nodes;
    std::unique_ptr<Token> token;
    Vector2 center;
    float radius;
    int nextNodeId{0};

    auto reorganizeNodes() -> void;

    bool ringFormationEnabled{true};

    Visualizer* visualizer{nullptr};

public:
    Ring(Vector2 center, float radius);

    // Rule of 5
    Ring(const Ring &other);                         // Copy constructor
    Ring(Ring &&other) noexcept;                     // Move constructor
    auto operator=(const Ring &other) -> Ring &;     // Copy assignment
    auto operator=(Ring &&other) noexcept -> Ring &; // Move assignment
    ~Ring() = default;

    auto setVisualizer(Visualizer* vis) -> void { visualizer = vis; }

    // Operator overloading
    auto operator+=(std::string nodeName) -> Ring &;
    auto operator-=(std::string_view nodeName) -> Ring &;
    auto operator[](size_t idx) -> Node &;
    auto operator[](size_t idx) const -> const Node &;
    auto operator*=(float scale) -> Ring &;

    auto addNode(std::string name) -> void;
    auto removeLastNode() -> void;
    auto spawnToken() -> void;
    auto update(float dt) -> void;

    auto getNodes() const -> const std::vector<std::unique_ptr<Node>> & { return nodes; }
    auto getToken() const -> const Token * { return token.get(); }
    auto getCenter() const -> Vector2 { return center; }
    auto getRadius() const -> float { return radius; }
    auto getNodeCount() const -> size_t { return nodes.size(); }
    auto updateNodeMovement(float dt, Vector2 bounds) -> void;
    auto handleNodeDragging(Vector2 mousePos, bool mousePressed, const Camera2D &camera) -> void;
    auto setAllNodesMobile(bool mobile) -> void;

    auto shouldReorganize() const -> bool;
    auto reorganizeFromPositions() -> void;

    auto applyRingFormationForces() -> void;
    auto calculateRingCenter() -> Vector2;

    auto setRingFormation(bool enabled) -> void { ringFormationEnabled = enabled; }
    auto getRingFormation() const -> bool { return ringFormationEnabled; }

    auto insertData(std::string key, std::string value) -> void;
    auto findDataOwner(int hash) -> Node *;
    auto repartitionData() -> void;
    auto assignTokenRanges() -> void;
    auto getDataDistribution() -> std::vector<int>; // Returns count per node

    auto forceRepartitionWithVisualization() -> void;

    // Message Routing System
    auto routeMessage(int startNodeId, std::unique_ptr<Node::RoutingMessage> msg) -> void;
    auto processMessageQueue(float dt) -> void;

    friend class Visualizer;

private:
    struct PendingMessage {
        std::unique_ptr<Node::RoutingMessage> content;
        int currentNodeId;
        int targetNodeId; // Next hop
        float progress; // 0.0 to 1.0
        Vector2 startPos;
        Vector2 endPos;
    };
    std::vector<PendingMessage> messageQueue;
    
    auto getNextNode(int currentNodeId) -> Node*;
