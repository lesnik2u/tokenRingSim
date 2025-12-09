#pragma once
#include "core/Node.h"
#include "core/Token.h"
#include "core/SpatialGrid.h"
#include <memory>
#include <raylib.h>
#include <vector>
#include "graphics/Visualizer.h"
#include "utils/Logger.h"

class Visualizer;
class SimulationManager;

/**
 * @brief Manages the Distributed Hash Table (DHT) ring.
 */
class Ring {
private:
    std::vector<std::unique_ptr<Node>> nodes;
    std::unique_ptr<Token> token;
    Vector2 center;
    float radius;
    int nextNodeId{0};
    int replicationFactor{3}; // Default replication factor

    auto reorganizeNodes() -> void;
    auto findNodeIndexById(int nodeId) const -> int; // Returns -1 if not found

    bool ringFormationEnabled{true};

    Visualizer* visualizer{nullptr};
    int ringId{-1}; // Unique ID for this ring, set by SimulationManager
    SimulationManager* simulationManager_ = nullptr; // Raw pointer to SimulationManager
    SpatialGrid spatialGrid{100.0f}; // Spatial grid for optimized neighbor queries
    float currentMaxVelocity{0.0f}; // Track max velocity for adaptive physics
    std::unordered_map<int, Node*> nodeIdMap; // Fast lookup O(1)

public:
    // Emergent Simulation Parameters
    int maxClusterSize{20};
    struct PhysicsParams {
        float searchRadius = 200.0f;
        float idealDist = 100.0f;
        float chainAttractStrength = 60.0f; // Very Strong
        float repulsionStrength = 300.0f;   // Very Strong
        float vortexStrength = 5.0f;
        float boundaryStrength = 1.0f;
        float splitStrength = 800.0f;       // Explosive
        float friction = 0.85f;             // High drag for stability
        float maxSpeed = 500.0f;            // Increased speed limit
    };
    PhysicsParams physics;

    Ring(Vector2 center, float radius);

    // Rule of 5
    Ring(const Ring &other);                         // Copy constructor
    Ring(Ring &&other) noexcept;                     // Move constructor
    auto operator=(const Ring &other) -> Ring &;     // Copy assignment
    auto operator=(Ring &&other) noexcept -> Ring &; // Move assignment
    ~Ring() = default;

    auto setVisualizer(Visualizer* vis) -> void { visualizer = vis; }
    auto setSimulationManager(SimulationManager* manager) -> void { simulationManager_ = manager; }

    // Operator overloading
    auto operator+=(std::string nodeName) -> Ring &;
    auto operator-=(std::string_view nodeName) -> Ring &;
    auto operator[](size_t idx) -> Node &;
    auto operator[](size_t idx) const -> const Node &;
    auto operator*=(float scale) -> Ring &;

    auto addNode(std::string name) -> void;
    auto removeLastNode() -> void;
    auto removeNode(int nodeId) -> void;
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

    auto applyRingFormationForces(float dt) -> void;
    auto resolveCollisions() -> void; // Hard collision resolution
    auto calculateRingCenter() -> Vector2;

    auto setRingFormation(bool enabled) -> void { ringFormationEnabled = enabled; }
    auto getRingFormation() const -> bool { return ringFormationEnabled; }
    auto getReplicationFactor() const -> int { return replicationFactor; }
    auto setReplicationFactor(int rf) -> void {
        if (rf > 0 && rf <= nodes.size()) { // Ensure RF is valid
            replicationFactor = rf;
            repartitionData(); // Re-replicate data if RF changes
        } else {
            APP_LOG_ERROR("Invalid replication factor: {} for ring with {} nodes", rf, nodes.size());
        }
    }

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

    int selectedNodeId = -1; // -1 indicates no node is selected

public:
    auto setSelectedNode(int id) -> void { selectedNodeId = id; }
    auto getSelectedNodeId() const -> int { return selectedNodeId; }
    auto getSelectedNode() const -> Node*;

    auto setRingId(int id) -> void { ringId = id; }
    auto getRingId() const -> int { return ringId; }
    
    auto setMaxClusterSize(int size) -> void { maxClusterSize = size; }
    auto getMaxClusterSize() const -> int { return maxClusterSize; }
};