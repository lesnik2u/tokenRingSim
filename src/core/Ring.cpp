#include "core/Ring.h"
#include "utils/Logger.h"
#include "graphics/Visualizer.h"
#include <algorithm>
#include <map>
#include <numbers>
#include <raylib.h>
#include <raymath.h>
#include <stdexcept>
#include <utility>

Ring::Ring(Vector2 center, float radius) : center(center), radius(radius) {
    LOG_INFO("Ring created at ({}, {}) with radius {}", center.x, center.y, radius);
}
// Copy constructor
Ring::Ring(const Ring &other)
    : center(other.center), radius(other.radius), nextNodeId(other.nextNodeId) {

    // Deep copy nodes
    for (const auto &node : other.nodes) {
        auto newNode =
            std::make_unique<Node>(node->getId(), std::string(node->getName()), node->getAngle());
        newNode->updatePosition(center, radius);
        nodes.push_back(std::move(newNode));
    }

    // Deep copy token if exists
    if (other.token) {
        token = std::make_unique<Token>(other.token->getId());
        token->moveToNextNode(other.token->getCurrentNodeId());

        // Restore hasToken flags
        for (auto &node : nodes) {
            if (node->getId() == token->getCurrentNodeId()) {
                node->hasToken = true;
            }
        }
    }
}

// Move constructor
Ring::Ring(Ring &&other) noexcept
    : nodes(std::move(other.nodes)), token(std::move(other.token)), center(other.center),
      radius(other.radius), nextNodeId(other.nextNodeId) {}

// Copy assignment
auto Ring::operator=(const Ring &other) -> Ring & {
    if (this != &other) {
        Ring temp(other); // Copy-and-swap idiom
        std::swap(nodes, temp.nodes);
        std::swap(token, temp.token);
        center = temp.center;
        radius = temp.radius;
        nextNodeId = temp.nextNodeId;
    }
    return *this;
}

// Move assignment
auto Ring::operator=(Ring &&other) noexcept -> Ring & {
    if (this != &other) {
        nodes = std::move(other.nodes);
        token = std::move(other.token);
        center = other.center;
        radius = other.radius;
        nextNodeId = other.nextNodeId;
    }
    return *this;
}

auto Ring::operator+=(std::string nodeName) -> Ring & {
    addNode(std::move(nodeName));
    return *this;
}

auto Ring::operator-=(std::string_view nodeName) -> Ring & {
    auto it =
        std::remove_if(nodes.begin(), nodes.end(), [nodeName](const std::unique_ptr<Node> &node) {
            return node->getName() == nodeName;
        });

    if (it != nodes.end()) {
        nodes.erase(it, nodes.end());
        reorganizeNodes();
    }
    return *this;
}

auto Ring::operator[](size_t idx) -> Node & {
    if (idx >= nodes.size()) {
        throw std::out_of_range("Node index out of range");
    }
    return *nodes[idx];
}

auto Ring::operator[](size_t idx) const -> const Node & {
    if (idx >= nodes.size()) {
        throw std::out_of_range("Node index out of range");
    }
    return *nodes[idx];
}

auto Ring::operator*=(float scale) -> Ring & {
    radius *= scale;
    reorganizeNodes();
    return *this;
}

auto Ring::addNode(std::string name) -> void {
    auto node = std::make_unique<Node>(nextNodeId++, std::move(name), 0.0f);
    LOG_DEBUG("Adding node: id={}, name={}", node->getId(), node->getName());
    nodes.push_back(std::move(node));
    reorganizeNodes();
    assignTokenRanges(); // Assign ranges immediately
    repartitionData();
}

auto Ring::removeLastNode() -> void {
    if (nodes.empty())
        return;

    if (nodes.size() <= 2) {
        LOG_ERROR("Cannot remove node: minimum 2 nodes required");
        return;
    }

    LOG_DEBUG("Removing node: {}", nodes.back()->getName());

    // First repartition to move data away from dying node
    repartitionData();

    // Then remove the node
    nodes.pop_back();
    reorganizeNodes();
    assignTokenRanges();
}

auto Ring::spawnToken() -> void {
    if (!nodes.empty() && !token) {
        token = std::make_unique<Token>(0);
        token->currentNodeId = nodes[0]->getId();
        nodes[0]->hasToken = true;
    }
}

auto Ring::update(float dt) -> void {
    if (!token || nodes.empty())
        return;

    // Only update travel if not at destination
    if (token->getTravelProgress() < 1.0f) {
        token->updateTravel(dt * 0.5f);
    }

    // Check if reached next node
    if (token->getTravelProgress() >= 1.0f) {
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i]->getId() == token->getCurrentNodeId()) {
                nodes[i]->hasToken = false;

                size_t nextIdx = (i + 1) % nodes.size();
                token->moveToNextNode(nodes[nextIdx]->getId());
                nodes[nextIdx]->hasToken = true;
                break;
            }
        }
    }
    
    processMessageQueue(dt);
}

auto Ring::reorganizeNodes() -> void {
    if (nodes.empty())
        return;

    float angleStep = (2.0f * std::numbers::pi_v<float>) / nodes.size();

    for (size_t i = 0; i < nodes.size(); ++i) {
        nodes[i]->setAngle(i * angleStep);
        nodes[i]->updatePosition(center, radius);
    }
}

auto Ring::updateNodeMovement(float dt, Vector2 bounds) -> void {
    for (auto &node : nodes) {
        node->moveFreely(dt, bounds);
    }
}

auto Ring::handleNodeDragging(Vector2 mousePos, bool mousePressed, const Camera2D &camera) -> void {
    // Convert screen space to world space
    Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);

    if (mousePressed) {
        // Check if clicking on a node
        for (auto &node : nodes) {
            Vector2 nodePos = node->getPosition();
            float distance = Vector2Distance(worldPos, nodePos);

            if (distance < 30.0f && !node->getDragging()) {
                node->setDragging(true);
                LOG_DEBUG("Started dragging node: {}", node->getName());
                break;
            }
        }
    } else {
        // Release all nodes
        for (auto &node : nodes) {
            if (node->getDragging()) {
                node->setDragging(false);
                LOG_DEBUG("Stopped dragging node: {}", node->getName());
            }
        }
    }

    // Update dragged node position
    for (auto &node : nodes) {
        if (node->getDragging()) {
            node->setPosition(worldPos);
        }
    }
}

auto Ring::setAllNodesMobile(bool mobile) -> void {
    for (auto &node : nodes) {
        node->setMobile(mobile);
        if (mobile && node->getVelocity().x == 0 && node->getVelocity().y == 0) {
            float vx = (GetRandomValue(-50, 50));
            float vy = (GetRandomValue(-50, 50));
            node->setVelocity(Vector2{vx, vy});
        }
    }

    if (!mobile) {
        reorganizeNodes();
    }

    LOG_INFO("Set all nodes mobile: {}", mobile);
}

auto Ring::shouldReorganize() const -> bool {
    // Check if any node is not mobile (all static nodes should be in ring)
    for (const auto &node : nodes) {
        if (!node->getMobile()) {
            return true;
        }
    }
    return false;
}

auto Ring::reorganizeFromPositions() -> void {
    if (nodes.empty())
        return;

    // Check if nodes should be in ring formation (not mobile)
    bool allStatic = true;
    for (const auto &node : nodes) {
        if (node->getMobile()) {
            allStatic = false;
            break;
        }
    }

    if (!allStatic) {
        // Nodes are mobile, calculate center and organize in ring
        Vector2 centerPos = {0, 0};
        for (const auto &node : nodes) {
            centerPos.x += node->getPosition().x;
            centerPos.y += node->getPosition().y;
        }
        centerPos.x /= nodes.size();
        centerPos.y /= nodes.size();

        // Sort nodes by angle from center
        std::sort(nodes.begin(), nodes.end(),
                  [centerPos](const std::unique_ptr<Node> &a, const std::unique_ptr<Node> &b) {
                      Vector2 aPos = a->getPosition();
                      Vector2 bPos = b->getPosition();
                      float angleA = atan2(aPos.y - centerPos.y, aPos.x - centerPos.x);
                      float angleB = atan2(bPos.y - centerPos.y, bPos.x - centerPos.x);
                      return angleA < angleB;
                  });

        // Update angles based on current positions
        for (size_t i = 0; i < nodes.size(); ++i) {
            Vector2 pos = nodes[i]->getPosition();
            float angle = atan2(pos.y - centerPos.y, pos.x - centerPos.x);
            nodes[i]->setAngle(angle);
        }

        LOG_DEBUG("Reorganized ring from positions");
    } else {
        // Static ring - use standard organization
        reorganizeNodes();
    }
}

auto Ring::calculateRingCenter() -> Vector2 {
    if (nodes.empty())
        return center;

    Vector2 avgPos = {0, 0};
    for (const auto &node : nodes) {
        avgPos.x += node->getPosition().x;
        avgPos.y += node->getPosition().y;
    }
    avgPos.x /= nodes.size();
    avgPos.y /= nodes.size();

    return avgPos;
}

auto Ring::applyRingFormationForces() -> void {
    if (nodes.empty() || !ringFormationEnabled)
        return;

    Vector2 ringCenter = calculateRingCenter();
    float targetRadius = radius;

    for (auto &node : nodes) {
        if (!node->getMobile())
            continue;

        Vector2 nodePos = node->getPosition();

        // Force toward ring center
        Vector2 toCenter = {ringCenter.x - nodePos.x, ringCenter.y - nodePos.y};
        float distToCenter = sqrt(toCenter.x * toCenter.x + toCenter.y * toCenter.y);

        if (distToCenter > 0.1f) {
            toCenter.x /= distToCenter;
            toCenter.y /= distToCenter;

            float radiusError = distToCenter - targetRadius;
            float springStrength = 30.0f;

            Vector2 springForce = {toCenter.x * radiusError * springStrength,
                                   toCenter.y * radiusError * springStrength};

            node->applyForce(springForce);
        }

        // Repulsion from other nodes
        for (const auto &other : nodes) {
            if (node.get() == other.get())
                continue;

            Vector2 otherPos = other->getPosition();
            Vector2 diff = {nodePos.x - otherPos.x, nodePos.y - otherPos.y};
            float dist = sqrt(diff.x * diff.x + diff.y * diff.y);

            float minDist = 80.0f;
            if (dist < minDist && dist > 0.1f) {
                diff.x /= dist;
                diff.y /= dist;

                float repulsionStrength = 300.0f / (dist * dist);
                Vector2 repulsion = {diff.x * repulsionStrength, diff.y * repulsionStrength};

                node->applyForce(repulsion);
            }
        }

        // Friction
        Vector2 vel = node->getVelocity();
        node->setVelocity(Vector2{vel.x * 0.98f, vel.y * 0.98f});
    }
}

auto Ring::insertData(std::string key, std::string value) -> void {
    if (nodes.empty()) return;

    // Start from a random node (simulating a client request hitting a random server)
    int startNodeIdx = GetRandomValue(0, static_cast<int>(nodes.size()) - 1);
    Node* startNode = nodes[startNodeIdx].get();

    auto data = std::make_unique<DataItem>(std::move(key), std::move(value));
    int hash = data->getHash();

    auto msg = std::make_unique<Node::RoutingMessage>();
    msg->data = std::move(data);
    msg->targetHash = hash;
    msg->isReplicationMessage = false;
    msg->ttl = static_cast<int>(nodes.size()) * 2;

    LOG_INFO("Client request: Insert '{}' (hash={}°) -> Node '{}'", msg->data->getKey(), hash, startNode->getName());

    // Try to process immediately at start node
    auto [accepted, forwardMsg] = startNode->receiveMessage(std::move(*msg));

    if (accepted) {
        LOG_INFO("Data '{}' stored immediately at Node '{}'", key, startNode->getName());
    } else {
        // Needs forwarding
        routeMessage(startNode->getId(), std::move(forwardMsg));
    }
}

auto Ring::routeMessage(int startNodeId, std::unique_ptr<Node::RoutingMessage> msg) -> void {
    Node* current = nullptr;
    for(auto& n : nodes) if(n->getId() == startNodeId) current = n.get();
    
    if(!current) return;

    Node* next = getNextNode(startNodeId);
    if(!next) return;

    PendingMessage pm;
    pm.content = std::move(msg);
    pm.currentNodeId = startNodeId;
    pm.targetNodeId = next->getId();
    pm.progress = 0.0f;
    pm.startPos = current->getPosition();
    pm.endPos = next->getPosition();
    
    messageQueue.push_back(std::move(pm));
    
    if (visualizer) {
         // Visualizer will just read from messageQueue? Or we trigger it? 
         // Ideally Visualizer should read this state, but for now let's keep using startDataTransfer for "visuals"
         // wait, messageQueue IS the visual state for routing.
    }
}

auto Ring::getNextNode(int currentNodeId) -> Node* {
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i]->getId() == currentNodeId) {
            size_t nextIdx = (i + 1) % nodes.size();
            return nodes[nextIdx].get();
        }
    }
    return nullptr;
}

auto Ring::processMessageQueue(float dt) -> void {
    for (auto it = messageQueue.begin(); it != messageQueue.end(); ) {
        it->progress += dt * 2.0f; // Speed of message travel

        // Update positions if nodes moved
        Node* start = nullptr; 
        Node* end = nullptr;
        for(auto& n : nodes) {
            if(n->getId() == it->currentNodeId) start = n.get();
            if(n->getId() == it->targetNodeId) end = n.get();
        }

        if(start && end) {
            it->startPos = start->getPosition();
            it->endPos = end->getPosition();
        }

        if (it->progress >= 1.0f) {
            // Arrived at next node
            Node* targetNode = end;
            if (targetNode) {
                auto [accepted, forwardMsg] = targetNode->receiveMessage(std::move(*it->content));
                
                if (accepted) {
                    // Message delivered!
                    // TODO: Trigger Replication here if it was a primary store
                    LOG_INFO("Message delivered to Node '{}'", targetNode->getName());
                    it = messageQueue.erase(it);
                } else {
                    // Needs to forward again
                    auto msg = std::move(forwardMsg);
                    int currentId = targetNode->getId();
                    it = messageQueue.erase(it); // remove current hop
                    routeMessage(currentId, std::move(msg)); // schedule next hop
                    continue; // loop again
                }
            } else {
                 it = messageQueue.erase(it);
            }
        } else {
            ++it;
        }
    }
}

auto Ring::findDataOwner(int hash) -> Node * {
    if (nodes.empty())
        return nullptr;

    // Find node responsible for this hash
    for (auto &node : nodes) {
        if (node->ownsHash(hash)) {
            return node.get();
        }
    }

    // Fallback to first node
    return nodes[0].get();
}

auto Ring::assignTokenRanges() -> void {
    if (nodes.empty())
        return;

    int rangeSize = 360 / nodes.size();

    for (size_t i = 0; i < nodes.size(); ++i) {
        int start = i * rangeSize;
        int end = (i + 1) * rangeSize;
        if (i == nodes.size() - 1) {
            end = 360; // Last node gets remainder
        }
        nodes[i]->setTokenRange(start, end);
        LOG_DEBUG("Node '{}' owns range [{}°, {}°)", nodes[i]->getName(), start, end);
    }
}

auto Ring::repartitionData() -> void {
    LOG_INFO("=== Starting data repartitioning ===");

    if (nodes.empty()) {
        LOG_ERROR("No nodes available for repartitioning");
        return;
    }

    // Map old owners with their positions
    std::map<std::string, std::pair<Node *, Vector2>> oldOwners;
    for (auto &node : nodes) {
        Vector2 pos = node->getPosition();
        LOG_DEBUG("Node '{}' position: ({}, {})", node->getName(), pos.x, pos.y);
        for (const auto &data : node->storedData) {
            oldOwners[data->getKey()] = {node.get(), pos};
        }
    }

    // Collect all data
    std::vector<std::unique_ptr<DataItem>> allData;
    for (auto &node : nodes) {
        while (node->getDataCount() > 0) {
            auto &nodeData = node->storedData;
            if (!nodeData.empty()) {
                allData.push_back(std::move(nodeData.back()));
                nodeData.pop_back();
            }
        }
    }

    LOG_INFO("Collected {} data items for redistribution", allData.size());

    assignTokenRanges();

    // Redistribute with visualization
    int transferCount = 0;
    for (auto &data : allData) {
        int hash = data->getHash();
        Node *newOwner = findDataOwner(hash);

        if (newOwner) {
            auto oldOwnerIt = oldOwners.find(data->getKey());

            if (oldOwnerIt != oldOwners.end()) {
                Node *oldOwner = oldOwnerIt->second.first;
                Vector2 oldPos = oldOwnerIt->second.second;
                Vector2 newPos = newOwner->getPosition();

                if (oldOwner != newOwner && visualizer) {
                    LOG_INFO(
                        "Visualizing transfer: '{}' from {} to {} (positions: ({},{}) -> ({},{}))",
                        data->getKey(), oldOwner->getName(), newOwner->getName(), oldPos.x,
                        oldPos.y, newPos.x, newPos.y);

                    visualizer->startDataTransfer(oldPos, newPos, data->getKey());
                    transferCount++;
                }
            }

            newOwner->addData(std::move(data));
        }
    }

    LOG_INFO("=== Repartitioning complete - {} transfers visualized ===", transferCount);
}

auto Ring::getDataDistribution() -> std::vector<int> {
    std::vector<int> distribution;
    for (const auto &node : nodes) {
        distribution.push_back(node->getDataCount());
    }
    return distribution;
}

auto Ring::forceRepartitionWithVisualization() -> void {
    LOG_INFO("=== Forcing visual repartitioning ===");

    if (nodes.empty() || !visualizer) {
        LOG_ERROR("Cannot visualize: no nodes or visualizer");
        return;
    }

    // For each node's data, show transfer animation to itself or neighbors
    for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& currentNode = nodes[i];
        for (const auto& data : currentNode->getStoredData()) {
            // Find correct owner
            Node* correctOwner = findDataOwner(data->getHash());

            if (correctOwner && correctOwner != currentNode.get()) {
                // Data is in wrong place, animate transfer
                visualizer->startDataTransfer(
                    currentNode->getPosition(),
                    correctOwner->getPosition(),
                    data->getKey()
                );
            }
        }
    }

    // Now do actual repartition
    repartitionData();
}
