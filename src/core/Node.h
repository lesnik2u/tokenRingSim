#pragma once
#include "Data.h"
#include "Entity.h"
#include <memory>
#include <raylib.h>
#include <unordered_map>
#include <vector>

class Ring;

class Node : public Entity {
private:
    Vector2 position;
    Vector2 velocity{0, 0};
    float angle;
    bool hasToken{false};
    bool isDragging{false};
    bool isMobile{false};
    bool activeStatus{true};
    bool isSelected{false};

    std::vector<std::unique_ptr<DataItem>> storedData;
    std::unordered_map<std::string, DataItem *> dataIndex;

    int tokenRangeStart{0};
    int tokenRangeEnd{0};

public:
    explicit Node(int id, std::string name, float angle);

    // Operators
    friend bool operator<(const Node &lhs, const Node &rhs);
    friend bool operator>(const Node &lhs, const Node &rhs);
    friend bool operator!=(const Node &lhs, const Node &rhs);
    friend bool operator==(const Node &lhs, const Node &rhs);

    // Getters
    Vector2 getPosition() const { return position; }
    float getAngle() const { return angle; }
    bool hasTokenPresent() const { return hasToken; }
    const std::string &getName() const { return name; }

    // Setters and Updates
    void setAngle(float newAngle);
    void updatePosition(Vector2 center, float radius);
    void setPosition(Vector2 pos) { position = pos; }
    Vector2 getVelocity() const { return velocity; }
    void setVelocity(Vector2 vel) { velocity = vel; }

    void setMobile(bool mobile) { isMobile = mobile; }
    bool getMobile() const { return isMobile; }

    void setDragging(bool dragging) { isDragging = dragging; }
    bool getDragging() const { return isDragging; }

    // Physics
    void moveFreely(float dt, Vector2 bounds);
    std::string toString() const override;
    void applyForce(Vector2 force, float dt, float maxSpeed);
    void resetForces();

    // Data Management
    void addData(std::unique_ptr<DataItem> data);
    void removeData(const std::string &key);
    std::vector<std::unique_ptr<DataItem>> clearData();
    bool hasData(const std::string &key);
    DataItem *getData(const std::string &key);
    size_t getDataCount() const { return storedData.size(); }
    const std::vector<std::unique_ptr<DataItem>> &getStoredData() const { return storedData; }

    struct RoutingMessage {
        std::unique_ptr<DataItem> data;
        int targetHash;
        bool isReplicationMessage;
        int ttl;
    };

    // Message Handling
    std::pair<bool, std::unique_ptr<RoutingMessage>> receiveMessage(RoutingMessage message);

    // Token and Hashing
    void setTokenRange(int start, int end);
    int getTokenRangeStart() const { return tokenRangeStart; }
    int getTokenRangeEnd() const { return tokenRangeEnd; }
    bool ownsHash(int hash) const;

    // State
    bool isActive() const { return activeStatus; }
    void setActive(bool active) { activeStatus = active; }
    bool getSelected() const { return isSelected; }
    void setSelected(bool selected) { isSelected = selected; }

    // Topology
    std::vector<Node *> neighbors;
    std::unordered_map<Node *, int> bondAges;

    void clearNeighbors() {
        neighbors.clear();
        bondAges.clear();
    }
    void addNeighbor(Node *node) {
        neighbors.push_back(node);
        bondAges[node] = 0;
    }
    void removeNeighbor(Node *node) {
        auto it = std::find(neighbors.begin(), neighbors.end(), node);
        if (it != neighbors.end()) {
            *it = neighbors.back();
            neighbors.pop_back();
            bondAges.erase(node);
        }
    }
    const std::vector<Node *> &getNeighbors() const { return neighbors; }

    void incrementBondAge(Node *neighbor) {
        if (bondAges.find(neighbor) != bondAges.end()) {
            bondAges[neighbor]++;
        }
    }

    void resetBondAge(Node *neighbor) {
        if (bondAges.find(neighbor) != bondAges.end()) {
            bondAges[neighbor] = 0;
        }
    }

    int getBondAge(Node *node) const {
        if (bondAges.find(node) != bondAges.end())
            return bondAges.at(node);
        return 0;
    }

    // Clustering
    int clusterId = -1;
    void setClusterId(int id) { clusterId = id; }
    int getClusterId() const { return clusterId; }

    int clusterSize = 1;
    void setClusterSize(int size) { clusterSize = size; }
    int getClusterSize() const { return clusterSize; }

    friend class Ring;
    friend class Visualizer;
};