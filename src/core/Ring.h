#pragma once
#include "core/Node.h"
#include "core/SpatialGrid.h"
#include "core/Token.h"
#include "graphics/Visualizer.h"
#include "utils/Logger.h"
#include <functional>
#include <memory>
#include <raylib.h>
#include <set>
#include <unordered_set>
#include <vector>

class Visualizer;

class Ring {
private:
    std::vector<std::unique_ptr<Node>> nodes;
    std::unique_ptr<Token> token;
    Vector2 center;
    float radius;
    int nextNodeId{0};
    int replicationFactor{3};

    void reorganizeNodes();

    bool ringFormationEnabled{true};

    Visualizer *visualizer{nullptr};
    int ringId{-1};
    std::function<void(int)> onNodeRemovedCallback;
    std::function<void(Node *)> onNodeAddedCallback;
    SpatialGrid spatialGrid{100.0f};
    float currentMaxVelocity{10.0f};
    std::unordered_map<int, Node *> nodeIdMap;
    bool topologyDirty{true};

    // Cluster tracking
    std::unordered_map<int, int> clusterSizes;
    std::unordered_map<int, std::vector<Node *>> clusterEnds;
    std::unordered_set<int> dirtyClusters;
    std::set<int> freeClusterIds;
    int globalNextClusterId{0};

    float timeSinceLastSteal{0.0f};
    bool spatialGridDirty{true};
    mutable std::vector<Node *> scratchBuffer;

    float sortingTimer{0.0f};
    float sortingInterval{0.2f};

    uint64_t topologyVersion{0};
    uint64_t selectionVersion{0};

    void markClusterDirty(int id) {
        if (id != -1 && !topologyDirty) {
            dirtyClusters.insert(id);
            topologyVersion++;
        }
    }

    void markTopologyDirty() {
        topologyDirty = true;
        dirtyClusters.clear();
        topologyVersion++;
    }

public:
    uint64_t getTopologyVersion() const { return topologyVersion; }
    uint64_t getSelectionVersion() const { return selectionVersion; }

    int maxClusterSize{20};
    struct PhysicsParams {
        float searchRadius = 200.0f;
        float idealDist = 100.0f;
        float chainAttractStrength = 60.0f;
        float repulsionStrength = 300.0f;
        float vortexStrength = 5.0f;
        float boundaryStrength = 1.0f;
        float splitStrength = 800.0f;
        float friction = 0.85f;
        float maxSpeed = 500.0f;
    };
    PhysicsParams physics;

    Ring(Vector2 center, float radius);

    // Rule of 5
    Ring(const Ring &other);
    Ring(Ring &&other) noexcept;
    Ring &operator=(const Ring &other);
    Ring &operator=(Ring &&other) noexcept;
    ~Ring() = default;

    void setVisualizer(Visualizer *vis) { visualizer = vis; }
    void setOnNodeRemovedCallback(std::function<void(int)> callback) {
        onNodeRemovedCallback = std::move(callback);
    }
    void setOnNodeAddedCallback(std::function<void(Node *)> callback) {
        onNodeAddedCallback = std::move(callback);
    }

    // Operators
    Ring &operator+=(std::string nodeName);
    Ring &operator-=(std::string_view nodeName);
    Node &operator[](size_t idx);
    const Node &operator[](size_t idx) const;
    Ring &operator*=(float scale);

    // Node Management
    void addNode(std::string name);
    void removeLastNode();
    void removeNode(int nodeId);
    void spawnToken();
    void update(float dt);

    // Accessors
    const std::vector<std::unique_ptr<Node>> &getNodes() const { return nodes; }
    const Token *getToken() const { return token.get(); }
    Vector2 getCenter() const { return center; }
    float getRadius() const { return radius; }
    size_t getNodeCount() const { return nodes.size(); }

    // Physics and Movement
    void updateNodeMovement(float dt, Vector2 bounds);
    void updateDraggedNodePositions(Vector2 worldPos);
    void releaseAllDraggedNodes();
    void setAllNodesMobile(bool mobile);

    bool shouldReorganize() const;
    void reorganizeFromPositions();
    void sortNodesAngularly();

    void applyRingFormationForces(float dt);
    void resolveCollisions();
    Vector2 calculateRingCenter();

    // Configuration
    void setRingFormation(bool enabled) { ringFormationEnabled = enabled; }
    bool getRingFormation() const { return ringFormationEnabled; }
    int getReplicationFactor() const { return replicationFactor; }
    void setReplicationFactor(int rf) {
        if (rf > 0 && rf <= nodes.size()) {
            replicationFactor = rf;
            repartitionData();
        } else {
            APP_LOG_ERROR("Invalid replication factor: {} for ring with {} nodes", rf,
                          nodes.size());
        }
    }

    // Data Operations
    void insertData(std::string key, std::string value);
    Node *findDataOwner(int hash);
    void repartitionData();
    void assignTokenRanges();
    std::vector<int> getDataDistribution();

    void forceRepartitionWithVisualization();

    // Routing
    void routeMessage(int startNodeId, std::unique_ptr<Node::RoutingMessage> msg);
    void processMessageQueue(float dt);

    friend class Visualizer;

private:
    struct PendingMessage {
        std::unique_ptr<Node::RoutingMessage> content;
        int currentNodeId;
        int targetNodeId;
        float progress;
        Vector2 startPos;
        Vector2 endPos;
    };
    std::vector<PendingMessage> messageQueue;

    Node *getNextNode(int currentNodeId);

    int selectedNodeId = -1;

public:
    void setSelectedNode(int id);
    int getSelectedNodeId() const { return selectedNodeId; }
    Node *getSelectedNode() const;

    void setRingId(int id) { ringId = id; }
    int getRingId() const { return ringId; }

    void setMaxClusterSize(int size) { maxClusterSize = size; }
    int getMaxClusterSize() const { return maxClusterSize; }

    Node *getNodeAt(Vector2 pos, float radius) const {
        scratchBuffer.clear();
        spatialGrid.query(pos, radius, scratchBuffer);

        Node *closest = nullptr;
        float minD = radius;
        for (Node *n : scratchBuffer) {
            float d = Vector2Distance(pos, n->getPosition());
            if (d < minD) {
                minD = d;
                closest = n;
            }
        }
        return closest;
    }
};