#include "Node.h"
#include <cmath>
#include <format>
#include <algorithm>

Node::Node(int id, std::string name, float angle)
    : Entity(id, std::move(name)), angle(angle), position{0, 0} {}

auto operator<(const Node& lhs, const Node& rhs) -> bool {
    return lhs.getId() < rhs.getId();
}

auto operator>(const Node& lhs, const Node& rhs) -> bool {
    return lhs.getId() > rhs.getId();
}

auto operator!=(const Node& lhs, const Node& rhs) -> bool {
    return !(lhs == rhs);
}

auto operator==(const Node& lhs, const Node& rhs) -> bool {
    return lhs.getId() == rhs.getId();
}

auto Node::moveFreely(float dt, Vector2 bounds) -> void {
    if (!isMobile || isDragging) return;

    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
}

auto Node::setAngle(float newAngle) -> void {
    angle = newAngle;
}

auto Node::updatePosition(Vector2 center, float radius) -> void {
    position.x = center.x + radius * std::cos(angle);
    position.y = center.y + radius * std::sin(angle);
}

auto Node::toString() const -> std::string {
    return std::format("Node[id={}, name={}, angle={:.2f}, hasToken={}]",
                       id, name, angle, hasToken);
}

auto Node::applyForce(Vector2 force, float dt) -> void {
    velocity.x += force.x * dt;
    velocity.y += force.y * dt;

    // Max speed cap (optimized)
    float speedSq = velocity.x * velocity.x + velocity.y * velocity.y;
    const float maxSpeed = 1000.0f;
    if (speedSq > maxSpeed * maxSpeed) {
        float speed = sqrt(speedSq);
        float scale = maxSpeed / speed;
        velocity.x *= scale;
        velocity.y *= scale;
    }
}
auto Node::resetForces() -> void {
    // Don't reset, just apply damping
}

#include "Data.h"

auto Node::addData(std::unique_ptr<DataItem> data) -> void {
    dataIndex[data->getKey()] = data.get();
    storedData.push_back(std::move(data));
}

auto Node::removeData(const std::string& key) -> void {
    if (dataIndex.erase(key)) {
        storedData.erase(
            std::remove_if(storedData.begin(), storedData.end(),
                [&key](const std::unique_ptr<DataItem>& item) {
                    return item->getKey() == key;
                }),
            storedData.end()
        );
    }
}

auto Node::clearData() -> std::vector<std::unique_ptr<DataItem>> {
    std::vector<std::unique_ptr<DataItem>> extracted;
    extracted.reserve(storedData.size());
    for(auto& item : storedData) extracted.push_back(std::move(item));
    storedData.clear();
    dataIndex.clear();
    return extracted;
}

auto Node::hasData(const std::string& key) -> bool {
    return dataIndex.find(key) != dataIndex.end();
}

auto Node::getData(const std::string& key) -> DataItem* {
    auto it = dataIndex.find(key);
    return (it != dataIndex.end()) ? it->second : nullptr;
}

auto Node::receiveMessage(RoutingMessage message) -> std::pair<bool, std::unique_ptr<RoutingMessage>> {
    if (message.ttl <= 0) {
        // TTL expired, drop message (or log error)
        return {true, nullptr}; 
    }

    message.ttl--;

    // Check if data is valid
    if (!message.data) {
        return {true, nullptr}; // Invalid message, drop it
    }

    // Case 1: Replication command (forced storage)
    if (message.isReplicationMessage) {
        message.data->setIsReplica(true);
        addData(std::move(message.data));
        return {true, nullptr}; // Consumed
    }

    // Case 2: We are the owner
    if (ownsHash(message.targetHash)) {
        message.data->setIsReplica(false);
        addData(std::move(message.data));
        // Successfully routed to owner!
        return {true, nullptr}; 
    }

    // Case 3: Not the owner, request forwarding
    return {false, std::make_unique<RoutingMessage>(std::move(message))};
}

auto Node::setTokenRange(int start, int end) -> void {
    tokenRangeStart = start;
    tokenRangeEnd = end;
}

auto Node::ownsHash(int hash) const -> bool {
    if (tokenRangeStart <= tokenRangeEnd) {
        // Normal range (e.g., 10 to 100)
        // Inclusive start, exclusive end
        return hash >= tokenRangeStart && hash < tokenRangeEnd;
    } else {
        // Wrap around case (e.g., 350 to 10)
        // Covers [350, 360) AND [0, 10)
        return hash >= tokenRangeStart || hash < tokenRangeEnd;
    }
}
