#pragma once
#include "Entity.h"
#include <memory>
#include <raylib.h>
#include <vector>
#include <unordered_map>
#include "Data.h"

class Ring;

class Node : public Entity {
private:
    Vector2 position;
    Vector2 velocity{0, 0};
    float angle;
    bool hasToken{false};
    bool isDragging{false};
    bool isMobile{false};
    bool activeStatus{true}; // Nodes are active by default
    bool isSelected{false};
    std::vector<std::unique_ptr<DataItem>> storedData;
    int tokenRangeStart{0}; // Angle in degrees
    int tokenRangeEnd{0};

public:
    explicit Node(int id, std::string name, float angle);

    friend auto operator<(const Node &lhs, const Node &rhs) -> bool;
    friend auto operator>(const Node &lhs, const Node &rhs) -> bool;
    friend auto operator!=(const Node &lhs, const Node &rhs) -> bool;
    friend auto operator==(const Node &lhs, const Node &rhs) -> bool;

    auto getPosition() const -> Vector2 { return position; }
    auto getAngle() const -> float { return angle; }
    auto hasTokenPresent() const -> bool { return hasToken; }
    
    // Optimized accessor
    auto getName() const -> const std::string& { return name; }

    auto setAngle(float newAngle) -> void;
    auto updatePosition(Vector2 center, float radius) -> void;

    auto setPosition(Vector2 pos) -> void { position = pos; }
    auto getVelocity() const -> Vector2 { return velocity; }
    auto setVelocity(Vector2 vel) -> void { velocity = vel; }
    auto setMobile(bool mobile) -> void { isMobile = mobile; }
    auto getMobile() const -> bool { return isMobile; }
    auto setDragging(bool dragging) -> void { isDragging = dragging; }
    auto getDragging() const -> bool { return isDragging; }

    auto moveFreely(float dt, Vector2 bounds) -> void;
    auto toString() const -> std::string override;

    auto applyForce(Vector2 force) -> void;
    auto resetForces() -> void;

    auto addData(std::unique_ptr<DataItem> data) -> void;
    auto removeData(const std::string &key) -> void;
    auto hasData(const std::string &key) -> bool;
    auto getDataCount() const -> size_t { return storedData.size(); }
    auto getStoredData() const -> const std::vector<std::unique_ptr<DataItem>> & {
        return storedData;
    }
    
    // Routing & Replication
    struct RoutingMessage {
        std::unique_ptr<DataItem> data;
        int targetHash;
        bool isReplicationMessage; // If true, we are forcing storage as a replica
        int ttl; // Time to live (hops) to prevent infinite loops
    };

    // Returns true if accepted (arrived at destination), false if it needs to be forwarded
    auto receiveMessage(RoutingMessage message) -> std::pair<bool, std::unique_ptr<RoutingMessage>>;

    auto setTokenRange(int start, int end) -> void;
    auto getTokenRangeStart() const -> int { return tokenRangeStart; }
    auto getTokenRangeEnd() const -> int { return tokenRangeEnd; }
    auto ownsHash(int hash) const -> bool;

    auto isActive() const -> bool { return activeStatus; }
    auto setActive(bool active) -> void { activeStatus = active; }

    auto getSelected() const -> bool { return isSelected; }
    auto setSelected(bool selected) -> void { isSelected = selected; }

    // Emergent Topology
    std::vector<Node*> neighbors;
    std::unordered_map<Node*, int> bondAges; // Track bond stability

    auto clearNeighbors() -> void { 
        neighbors.clear(); 
        bondAges.clear();
    }
    auto addNeighbor(Node* node) -> void { 
        neighbors.push_back(node); 
        bondAges[node] = 0;
    }
    auto removeNeighbor(Node* node) -> void { 
        std::erase(neighbors, node); 
        bondAges.erase(node);
    }
    auto getNeighbors() const -> const std::vector<Node*>& { return neighbors; }
    
    auto incrementBondAges() -> void {
        for (auto& [node, age] : bondAges) {
            age++;
        }
    }
    auto getBondAge(Node* node) const -> int {
        if (bondAges.find(node) != bondAges.end()) return bondAges.at(node);
        return 0;
    }

    // Cluster/Ring ID for logic
    int clusterId = -1;
    auto setClusterId(int id) -> void { clusterId = id; }
    auto getClusterId() const -> int { return clusterId; }

    int clusterSize = 1;
    auto setClusterSize(int size) -> void { clusterSize = size; }
    auto getClusterSize() const -> int { return clusterSize; }

    friend class Ring;
    friend class Visualizer;
};
