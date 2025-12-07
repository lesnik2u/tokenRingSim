#include "core/Ring.h"
#include "utils/Logger.h"
#include "SimulationManager.h" // Added include for SimulationManager
#include "graphics/Visualizer.h"
#include <algorithm>
#include <map>
#include <numbers>
#include <raylib.h>
#include <raymath.h>
#include <stdexcept>
#include <utility>

Ring::Ring(Vector2 center, float radius) : center(center), radius(radius) {
    APP_LOG_INFO("Ring created at ({}, {}) with radius {}", center.x, center.y, radius);
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
    APP_LOG_DEBUG("Adding node: id={}, name={}", node->getId(), node->getName());
    nodes.push_back(std::move(node));
    reorganizeNodes();
    assignTokenRanges(); // Assign ranges immediately
    repartitionData();
}

auto Ring::removeLastNode() -> void {
    if (nodes.empty())
        return;

    if (nodes.size() <= 2) {
        APP_LOG_ERROR("Cannot remove node: minimum 2 nodes required");
        return;
    }

    APP_LOG_DEBUG("Removing node: {}", nodes.back()->getName());

    // First repartition to move data away from dying node
    repartitionData();

    // Then remove the node
    // Notify SimulationManager that node is being removed
    if (simulationManager_) {
        simulationManager_->onNodeRemoved(nodes.back()->getId());
    }
    nodes.pop_back();
    reorganizeNodes();
    assignTokenRanges();
}

auto Ring::removeNode(int nodeId) -> void {
    auto it = std::find_if(nodes.begin(), nodes.end(), [nodeId](const std::unique_ptr<Node>& n){
        return n->getId() == nodeId;
    });

    if (it != nodes.end()) {
        // Store the node being removed temporarily to access its properties safely
        // after `find_if` but before `erase` invalidates iterators.
        Node* nodeToRemove = it->get();

        // Clean up message queue - remove any messages involving this node
        messageQueue.erase(
            std::remove_if(messageQueue.begin(), messageQueue.end(),
                [nodeId](const PendingMessage& msg) {
                    return msg.currentNodeId == nodeId || msg.targetNodeId == nodeId;
                }),
            messageQueue.end()
        );
        
        // If the removed node was the token holder, move the token to the next node
        if (token && nodeToRemove->getId() == token->getCurrentNodeId()) {
            // Find the current index of the node to be removed
            size_t currentIdx = std::distance(nodes.begin(), it);

            if (nodes.size() > 1) { // If there are other nodes remaining
                size_t nextIdx = (currentIdx + 1) % nodes.size();
                // Ensure the token moves to a valid node after 'it' is removed.
                // If 'it' is the last element and nextIdx becomes 0, it means the token moves to the new first node.
                // If 'it' is not the last element, nextIdx will be 'currentIdx + 1'
                // The actual node at nextIdx will shift left by 1 when 'it' is erased if nextIdx > currentIdx.
                // So, if nextIdx is calculated *before* erase, and it refers to an element *after* 'it',
                // we just need to ensure the token goes to the correct node after the erase.
                
                // Search for the next active node by iterating from the element *after* the one to be removed, wrapping around if necessary
                auto search_token_it = std::next(it);
                if (search_token_it == nodes.end()) { // Wrap around
                    search_token_it = nodes.begin();
                }

                Node* nextActiveNode = nullptr;
                bool foundNextActive = false;
                auto start_search_token_it = search_token_it; // To detect if we've looped entirely
                do {
                    if (search_token_it->get()->isActive() && search_token_it->get()->getId() != nodeToRemove->getId()) {
                        nextActiveNode = search_token_it->get();
                        foundNextActive = true;
                        break;
                    }
                    std::advance(search_token_it, 1);
                    if (search_token_it == nodes.end()) {
                        search_token_it = nodes.begin();
                    }
                } while (search_token_it != start_search_token_it);


                if (foundNextActive && nextActiveNode) { // Ensure we actually found a different active node
                    token->moveToNextNode(nextActiveNode->getId());
                    nextActiveNode->hasToken = true;
                    nodeToRemove->hasToken = false; // The node being removed no longer has the token
                    APP_LOG_INFO("Token transferred from node {} to node {}", nodeId, nextActiveNode->getId());
                } else {
                    // No other active nodes, destroy the token
                    token.reset();
                    APP_LOG_INFO("Token invalidated as no other active nodes found after node {}", nodeId);
                }
            } else { // Only one node left, and it's being removed
                token.reset(); // Invalidate the token
                APP_LOG_INFO("Token invalidated as the last node {} is removed", nodeId);
            }
        }
        
        // Repartition data away from the dying node *before* it is removed
        // This ensures data is moved from the node before its unique_ptr is destructed.
        assignTokenRanges(); // Update ranges temporarily with current node count
        repartitionData(); // Redistribute data based on these ranges

        // Notify SimulationManager that node is being removed
        if (simulationManager_) {
            simulationManager_->onNodeRemoved(nodeId);
        }
        nodes.erase(it); // Erase the node here

        // After removal, re-organize and re-assign token ranges for the new set of nodes
        reorganizeNodes();
        assignTokenRanges();

        // If the token was removed and not reassigned (e.g., last node removed), spawn a new one if possible
        if (!token && !nodes.empty()) {
            spawnToken();
            APP_LOG_INFO("New token spawned after node removal.");
        }
        
        APP_LOG_INFO("Node with ID {} removed from Ring {}", nodeId, ringId);
    } else {
        APP_LOG_ERROR("Attempted to remove non-existent Node with ID {} from Ring {}", nodeId, ringId);
    }
}

auto Ring::spawnToken() -> void {
    if (!nodes.empty() && !token) {
        token = std::make_unique<Token>(0);
        token->currentNodeId = nodes[0]->getId();
        nodes[0]->hasToken = true;
    }
}

auto Ring::update(float dt) -> void {
    // Update Spatial Grid
    spatialGrid.clear();
    for (const auto& node : nodes) {
        spatialGrid.insert(node.get());
    }

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

auto Ring::findNodeIndexById(int nodeId) const -> int {
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i]->getId() == nodeId) {
            return static_cast<int>(i);
        }
    }
    return -1;
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
                APP_LOG_DEBUG("Started dragging node: {}", node->getName());
                break;
            }
        }
    } else {
        // Release all nodes
        bool wasDragging = false;
        for (auto &node : nodes) {
            if (node->getDragging()) {
                node->setDragging(false);
                APP_LOG_DEBUG("Stopped dragging node: {}", node->getName());
                wasDragging = true;
            }
        }
        if (wasDragging && !nodes.empty()) {
            // Always reorganize after dragging to connect to closest nodes
            reorganizeFromPositions();
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

    APP_LOG_INFO("Set all nodes mobile: {}", mobile);
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

    // Calculate center from current node positions
    Vector2 centerPos = {0, 0};
    for (const auto &node : nodes) {
        centerPos.x += node->getPosition().x;
        centerPos.y += node->getPosition().y;
    }
    centerPos.x /= nodes.size();
    centerPos.y /= nodes.size();

    // Sort nodes by angle from center (clockwise order)
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

    APP_LOG_INFO("Reorganized ring: nodes sorted by position to connect closest neighbors");
    assignTokenRanges(); // Update token ranges after re-sorting
    repartitionData();   // Re-distribute data based on new ranges
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

    // Flocking Parameters
    float separationRadius = 40.0f;
    float alignmentRadius = 80.0f;
    float cohesionRadius = 80.0f;
    
    float separationWeight = 2.5f;
    float alignmentWeight = 1.0f;
    float cohesionWeight = 1.0f;
    float centerAttractWeight = 0.5f;
    float tangentFlowWeight = 0.8f;

    Vector2 ringCenter = calculateRingCenter(); // Use dynamic center of mass or fixed 'center'

    for (auto &node : nodes) {
        if (!node->getMobile()) continue;

        Vector2 pos = node->getPosition();
        Vector2 vel = node->getVelocity();
        
        Vector2 separation = {0, 0};
        Vector2 alignment = {0, 0};
        Vector2 cohesion = {0, 0};
        int separationCount = 0;
        int alignmentCount = 0;
        int cohesionCount = 0;

        // Query neighbors from grid
        auto neighbors = spatialGrid.query(pos);

        for (Node* other : neighbors) {
            if (node.get() == other) continue;

            Vector2 otherPos = other->getPosition();
            float d = Vector2Distance(pos, otherPos);

            // Separation
            if (d > 0 && d < separationRadius) {
                Vector2 diff = Vector2Subtract(pos, otherPos);
                diff = Vector2Normalize(diff);
                diff = Vector2Scale(diff, 1.0f / d); // Weight by inverse distance
                separation = Vector2Add(separation, diff);
                separationCount++;
            }

            // Alignment
            if (d > 0 && d < alignmentRadius) {
                alignment = Vector2Add(alignment, other->getVelocity());
                alignmentCount++;
            }

            // Cohesion
            if (d > 0 && d < cohesionRadius) {
                cohesion = Vector2Add(cohesion, otherPos);
                cohesionCount++;
            }
        }

        // Apply Flocking Forces
        if (separationCount > 0) {
            separation = Vector2Scale(separation, 1.0f / separationCount);
            separation = Vector2Normalize(separation);
            separation = Vector2Scale(separation, 100.0f); // Max speed
            Vector2 steer = Vector2Subtract(separation, vel);
            // Limit steer? node->applyForce handles it implicitly via physics update usually
            node->applyForce(Vector2Scale(steer, separationWeight));
        }

        if (alignmentCount > 0) {
            alignment = Vector2Scale(alignment, 1.0f / alignmentCount);
            alignment = Vector2Normalize(alignment);
            alignment = Vector2Scale(alignment, 100.0f);
            Vector2 steer = Vector2Subtract(alignment, vel);
            node->applyForce(Vector2Scale(steer, alignmentWeight));
        }

        if (cohesionCount > 0) {
            cohesion = Vector2Scale(cohesion, 1.0f / cohesionCount); // Center of mass
            Vector2 desired = Vector2Subtract(cohesion, pos);
            desired = Vector2Normalize(desired);
            desired = Vector2Scale(desired, 100.0f);
            Vector2 steer = Vector2Subtract(desired, vel);
            node->applyForce(Vector2Scale(steer, cohesionWeight));
        }

        // --- Emergent Ring Formation Forces ---

        // 1. Center Attraction (keeps them in the play area)
        // Attract to the Ring's defined center, not just local center of mass
        Vector2 toCenter = Vector2Subtract(this->center, pos); // Use this->center for stability
        float distToCenter = Vector2Length(toCenter);
        if (distToCenter > radius * 0.5f) { // Allow some freedom in the middle
             toCenter = Vector2Normalize(toCenter);
             node->applyForce(Vector2Scale(toCenter, centerAttractWeight * 10.0f));
        }

        // 2. Tangential Flow (encourages circular motion / ring formation)
        // Vector pointing from center to node
        Vector2 radial = Vector2Subtract(pos, this->center);
        // Tangent is perpendicular to radial (-y, x)
        Vector2 tangent = {-radial.y, radial.x};
        tangent = Vector2Normalize(tangent);
        
        // Scale tangent force by distance from center (faster on outside?)
        node->applyForce(Vector2Scale(tangent, tangentFlowWeight * 20.0f));
        
        // Friction / Damping
        node->setVelocity(Vector2Scale(node->getVelocity(), 0.99f));
    }
}

auto Ring::insertData(std::string key, std::string value) -> void {
    if (nodes.empty()) return;

    // Store copies for later use (key and value will be moved)
    std::string keyCopy = key;
    std::string valueCopy = value;

    // Start from a random node (simulating a client request hitting a random server)
    int startNodeIdx = GetRandomValue(0, static_cast<int>(nodes.size()) - 1);
    Node* startNode = nodes[startNodeIdx].get();

    auto data = std::make_unique<DataItem>(std::move(key), std::move(value));
    int hash = data->getHash();

    APP_LOG_INFO("Client request: Insert '{}' (hash={}°) -> Node '{}'", keyCopy, hash, startNode->getName());

    auto msg = std::make_unique<Node::RoutingMessage>();
    msg->data = std::move(data);
    msg->targetHash = hash;
    msg->isReplicationMessage = false;
    msg->ttl = static_cast<int>(nodes.size()) * 2;

    // Try to process immediately at start node
    auto [accepted, forwardMsg] = startNode->receiveMessage(std::move(*msg));

    if (accepted) {
        APP_LOG_INFO("Data '{}' stored immediately at Node '{}'", keyCopy, startNode->getName());
        // No visualization here - data was stored immediately at start node
    } else {
        // Needs forwarding - visualization will happen in processMessageQueue
        routeMessage(startNode->getId(), std::move(forwardMsg));
    }

    // Handle replication
    int currentRF = replicationFactor;
    if (currentRF > nodes.size()) { // Cap RF at total node count
        currentRF = static_cast<int>(nodes.size());
    }
    
    // Find the primary owner node's index
    Node* primaryOwner = findDataOwner(hash);
    if (!primaryOwner) {
        APP_LOG_ERROR("Primary owner node not found for hash: {}", hash);
        return;
    }
    
    int primaryOwnerNodeId = primaryOwner->getId();
    int primaryOwnerIndex = findNodeIndexById(primaryOwnerNodeId);

    if (primaryOwnerIndex == -1) {
        APP_LOG_ERROR("Primary owner node not found for replication: {}", primaryOwnerNodeId);
        return;
    }

    Node* currentNode = nodes[primaryOwnerIndex].get();

    for (int i = 1; i < currentRF; ++i) { // Start from 1 as primary is already handled
        size_t nextIdx = (primaryOwnerIndex + i) % nodes.size();
        Node* replicaNode = nodes[nextIdx].get();

        auto replicaData = std::make_unique<DataItem>(keyCopy, valueCopy, true); // Mark as replica
        auto replicaMsg = std::make_unique<Node::RoutingMessage>();
        replicaMsg->data = std::move(replicaData);
        replicaMsg->targetHash = hash;
        replicaMsg->isReplicationMessage = true; // This is a replica message
        replicaMsg->ttl = static_cast<int>(nodes.size()) * 2; // TTL for replication messages

        // Route the replica message from the primary owner to the replica node
        // This simulates the primary owner sending the replica to the next node
        routeMessage(currentNode->getId(), std::move(replicaMsg));
        
        APP_LOG_INFO("Replicating '{}' to Node '{}' (hop {})", keyCopy, replicaNode->getName(), i);
        // Visualization will happen hop-by-hop in processMessageQueue

        // Update current node for next replica, ensuring local communication
        currentNode = replicaNode;
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

        // Check if nodes still exist (might have been removed)
        if (!start || !end) {
            APP_LOG_ERROR("Message queue contains reference to removed node");
            it = messageQueue.erase(it);
            continue;
        }

        it->startPos = start->getPosition();
        it->endPos = end->getPosition();

        if (it->progress >= 1.0f) {
            // Arrived at next node - visualize this hop
            if (visualizer) {
                bool isReplica = it->content->isReplicationMessage;
                visualizer->startDataTransfer(it->startPos, it->endPos, it->content->data->getKey(), isReplica);
            }
            
            Node* targetNode = end;
            auto [accepted, forwardMsg] = targetNode->receiveMessage(std::move(*it->content));
            
            if (accepted) {
                // Message delivered!
                APP_LOG_INFO("Message delivered to Node '{}'", targetNode->getName());
                it = messageQueue.erase(it);
            } else {
                // Needs to forward again
                auto msg = std::move(forwardMsg);
                int currentId = targetNode->getId();
                it = messageQueue.erase(it); // remove current hop
                routeMessage(currentId, std::move(msg)); // schedule next hop
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
        APP_LOG_DEBUG("Node '{}' owns range [{}°, {}°)", nodes[i]->getName(), start, end);
    }
}

auto Ring::repartitionData() -> void {
    APP_LOG_INFO("=== Starting data repartitioning ===");

    if (nodes.empty()) {
        APP_LOG_ERROR("No nodes available for repartitioning");
        return;
    }

    // Map old owners with their positions
    std::map<std::string, std::pair<Node *, Vector2>> oldOwners;
    for (auto &node : nodes) {
        Vector2 pos = node->getPosition();
        APP_LOG_DEBUG("Node '{}' position: ({}, {})", node->getName(), pos.x, pos.y);
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

    APP_LOG_INFO("Collected {} data items for redistribution", allData.size());

    assignTokenRanges();

    // Redistribute with visualization
    int transferCount = 0;
    for (auto &data : allData) {
        int hash = data->getHash();
        Node *newOwner = findDataOwner(hash);

        if (!newOwner) {
            APP_LOG_ERROR("Could not find owner for data '{}' with hash {}", data->getKey(), hash);
            continue;
        }

        auto oldOwnerIt = oldOwners.find(data->getKey());

        if (oldOwnerIt != oldOwners.end()) {
            Node *oldOwner = oldOwnerIt->second.first;
            Vector2 oldPos = oldOwnerIt->second.second;
            Vector2 newPos = newOwner->getPosition();

            if (oldOwner != newOwner && visualizer) {
                visualizer->startDataTransfer(oldPos, newPos, data->getKey(), false);
                transferCount++;
            }
        }

        newOwner->addData(std::move(data));
    }

    APP_LOG_INFO("=== Repartitioning complete - {} transfers visualized ===", transferCount);
}

auto Ring::getDataDistribution() -> std::vector<int> {
    std::vector<int> distribution;
    for (const auto &node : nodes) {
        distribution.push_back(node->getDataCount());
    }
    return distribution;
}

auto Ring::forceRepartitionWithVisualization() -> void {
    APP_LOG_INFO("=== Forcing visual repartitioning ===");

    if (nodes.empty() || !visualizer) {
        APP_LOG_ERROR("Cannot visualize: no nodes or visualizer");
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
                    data->getKey(),
                    false // Not a replica message
                );
            }
        }
    }

    // Now do actual repartition
    repartitionData();
}

auto Ring::getSelectedNode() const -> Node* {
    if (selectedNodeId == -1) {
        return nullptr;
    }
    for (const auto& node : nodes) {
        if (node->getId() == selectedNodeId) {
            return node.get();
        }
    }
    return nullptr;
}

