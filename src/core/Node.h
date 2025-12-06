#pragma once
#include "Entity.h"
#include <memory>
#include <raylib.h>
#include <vector>
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

    friend class Ring;
    friend class Visualizer;
};
