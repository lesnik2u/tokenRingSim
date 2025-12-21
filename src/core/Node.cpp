#include "Node.h"
#include "Data.h"
#include <cmath>
#include <format>
#include <raymath.h>

Node::Node(int id, std::string name, float angle)
    : Entity(id, std::move(name)), position({0, 0}), angle(angle) {}

bool operator<(const Node &lhs, const Node &rhs) { return lhs.getId() < rhs.getId(); }

bool operator>(const Node &lhs, const Node &rhs) { return lhs.getId() > rhs.getId(); }

bool operator!=(const Node &lhs, const Node &rhs) { return !(lhs == rhs); }

bool operator==(const Node &lhs, const Node &rhs) { return lhs.getId() == rhs.getId(); }

void Node::moveFreely(float dt, Vector2 bounds) {
    if (!isMobile || isDragging)
        return;

    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
}

void Node::setAngle(float newAngle) { angle = newAngle; }

void Node::updatePosition(Vector2 center, float radius) {
    position.x = center.x + radius * std::cos(angle);
    position.y = center.y + radius * std::sin(angle);
}

std::string Node::toString() const {
    return std::format("Node[id={}, name={}, angle={:.2f}, hasToken={}]", id, name, angle,
                       hasToken);
}

void Node::applyForce(Vector2 force, float dt, float maxSpeed) {
    if (!isMobile)
        return;

    velocity.x += force.x * dt;
    velocity.y += force.y * dt;

    float speedSq = Vector2LengthSqr(velocity);
    if (speedSq > maxSpeed * maxSpeed) {
        float speed = sqrtf(speedSq);
        velocity = Vector2Scale(velocity, maxSpeed / speed);
    }
}

void Node::resetForces() {}

void Node::addData(std::unique_ptr<DataItem> data) {
    dataIndex[data->getKey()] = data.get();
    storedData.push_back(std::move(data));
}

void Node::removeData(const std::string &key) {
    if (dataIndex.erase(key)) {
        storedData.erase(std::remove_if(storedData.begin(), storedData.end(),
                                        [&key](const std::unique_ptr<DataItem> &item) {
                                            return item->getKey() == key;
                                        }),
                         storedData.end());
    }
}

std::vector<std::unique_ptr<DataItem>> Node::clearData() {
    std::vector<std::unique_ptr<DataItem>> extracted;
    extracted.reserve(storedData.size());
    for (auto &item : storedData)
        extracted.push_back(std::move(item));
    storedData.clear();
    dataIndex.clear();
    return extracted;
}

bool Node::hasData(const std::string &key) { return dataIndex.find(key) != dataIndex.end(); }

DataItem *Node::getData(const std::string &key) {
    auto it = dataIndex.find(key);
    return (it != dataIndex.end()) ? it->second : nullptr;
}

std::pair<bool, std::unique_ptr<Node::RoutingMessage>>
Node::receiveMessage(RoutingMessage message) {
    if (message.ttl <= 0) {
        return {true, nullptr};
    }

    message.ttl--;

    if (!message.data) {
        return {true, nullptr};
    }

    if (message.isReplicationMessage) {
        message.data->setIsReplica(true);
        addData(std::move(message.data));
        return {true, nullptr};
    }

    if (ownsHash(message.targetHash)) {
        message.data->setIsReplica(false);
        addData(std::move(message.data));
        return {true, nullptr};
    }

    return {false, std::make_unique<RoutingMessage>(std::move(message))};
}

void Node::setTokenRange(int start, int end) {
    tokenRangeStart = start;
    tokenRangeEnd = end;
}

bool Node::ownsHash(int hash) const {
    if (tokenRangeStart <= tokenRangeEnd) {
        return hash >= tokenRangeStart && hash < tokenRangeEnd;
    } else {
        return hash >= tokenRangeStart || hash < tokenRangeEnd;
    }
}