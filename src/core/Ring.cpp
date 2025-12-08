#include "core/Ring.h"
#include "utils/Logger.h"
#include "SimulationManager.h"
#include "graphics/Visualizer.h"
#include <algorithm>
#include <map>
#include <numbers>
#include <raylib.h>
#include <raymath.h>
#include <stdexcept>
#include <utility>
#include <queue>
#include <unordered_map>

Ring::Ring(Vector2 center, float radius) : center(center), radius(radius) {
    APP_LOG_INFO("Ring created at ({}, {}) with radius {}", center.x, center.y, radius);
}

Ring::Ring(const Ring &other)
    : center(other.center), radius(other.radius), nextNodeId(other.nextNodeId) {
    for (const auto &node : other.nodes) {
        auto newNode =
            std::make_unique<Node>(node->getId(), std::string(node->getName()), node->getAngle());
        newNode->updatePosition(center, radius);
        nodes.push_back(std::move(newNode));
    }
    
    // Rebuild lookup map
    for (const auto& node : nodes) {
        nodeIdMap[node->getId()] = node.get();
    }

    if (other.token) {
        token = std::make_unique<Token>(other.token->getId());
        token->moveToNextNode(other.token->getCurrentNodeId());
        if (nodeIdMap.count(token->getCurrentNodeId())) {
            nodeIdMap[token->getCurrentNodeId()]->hasToken = true;
        }
    }
}

Ring::Ring(Ring &&other) noexcept
    : nodes(std::move(other.nodes)), token(std::move(other.token)), center(other.center),
      radius(other.radius), nextNodeId(other.nextNodeId), nodeIdMap(std::move(other.nodeIdMap)) {}

auto Ring::operator=(const Ring &other) -> Ring & {
    if (this != &other) {
        Ring temp(other);
        std::swap(nodes, temp.nodes);
        std::swap(token, temp.token);
        center = temp.center;
        radius = temp.radius;
        nextNodeId = temp.nextNodeId;
        std::swap(nodeIdMap, temp.nodeIdMap);
    }
    return *this;
}

auto Ring::operator=(Ring &&other) noexcept -> Ring & {
    if (this != &other) {
        nodes = std::move(other.nodes);
        token = std::move(other.token);
        center = other.center;
        radius = other.radius;
        nextNodeId = other.nextNodeId;
        nodeIdMap = std::move(other.nodeIdMap);
    }
    return *this;
}

auto Ring::operator+=(std::string nodeName) -> Ring & {
    addNode(std::move(nodeName));
    return *this;
}

auto Ring::operator-=(std::string_view nodeName) -> Ring & {
    // This still needs iteration unless name map exists
    auto it =
        std::remove_if(nodes.begin(), nodes.end(), [nodeName, this](const std::unique_ptr<Node> &node) {
            if (node->getName() == nodeName) {
                nodeIdMap.erase(node->getId());
                return true;
            }
            return false;
        });

    if (it != nodes.end()) {
        nodes.erase(it, nodes.end());
        reorganizeNodes();
    }
    return *this;
}

auto Ring::operator[](size_t idx) -> Node & {
    if (idx >= nodes.size()) throw std::out_of_range("Node index out of range");
    return *nodes[idx];
}

auto Ring::operator[](size_t idx) const -> const Node & {
    if (idx >= nodes.size()) throw std::out_of_range("Node index out of range");
    return *nodes[idx];
}

auto Ring::operator*=(float scale) -> Ring & {
    radius *= scale;
    reorganizeNodes();
    return *this;
}

auto Ring::addNode(std::string name) -> void {
    auto node = std::make_unique<Node>(nextNodeId++, std::move(name), 0.0f);
    node->setMobile(true);
    APP_LOG_DEBUG("Adding node: id={}, name={}", node->getId(), node->getName());
    
    // Add to map before moving
    Node* ptr = node.get();
    nodes.push_back(std::move(node));
    nodeIdMap[ptr->getId()] = ptr;
    
    assignTokenRanges();
    repartitionData();
}

auto Ring::removeLastNode() -> void {
    if (nodes.empty()) return;
    Node* node = nodes.back().get();
    
    // Cleanup neighbors before deletion
    for (Node* neighbor : node->getNeighbors()) {
        neighbor->removeNeighbor(node);
    }
    
    APP_LOG_DEBUG("Removing node: {}", node->getName());
    
    nodeIdMap.erase(node->getId()); // Remove from map
    
    repartitionData();
    if (simulationManager_) simulationManager_->onNodeRemoved(node->getId());
    nodes.pop_back();
    assignTokenRanges();
}

auto Ring::removeNode(int nodeId) -> void {
    // Still need linear search to remove from vector efficiently?
    // Or use map to find it, then swap-remove? 
    // Vector erase needs iterator.
    
    auto it = std::find_if(nodes.begin(), nodes.end(), [nodeId](const std::unique_ptr<Node>& n){
        return n->getId() == nodeId;
    });

    if (it != nodes.end()) {
        Node* nodeToRemove = it->get();
        
        for (Node* neighbor : nodeToRemove->getNeighbors()) {
            neighbor->removeNeighbor(nodeToRemove);
        }

        messageQueue.erase(
            std::remove_if(messageQueue.begin(), messageQueue.end(),
                [nodeId](const PendingMessage& msg) {
                    return msg.currentNodeId == nodeId || msg.targetNodeId == nodeId;
                }),
            messageQueue.end()
        );
        
        if (token && nodeToRemove->getId() == token->getCurrentNodeId()) {
            // Token logic (O(N) search for next node in vector order, but emergent logic uses neighbors)
            // We need to move token safely.
            
            // Just pick a neighbor?
            Node* nextActiveNode = nullptr;
            if (!nodeToRemove->getNeighbors().empty()) {
                // Use map lookup for neighbor ID? Neighbors are Node* now.
                nextActiveNode = nodeToRemove->getNeighbors()[0];
            }
            
            // Fallback scan if isolated
            if (!nextActiveNode) {
                 for(auto& n : nodes) {
                     if(n->getId() != nodeId) { nextActiveNode = n.get(); break; }
                 }
            }

            if (nextActiveNode) {
                token->moveToNextNode(nextActiveNode->getId());
                nextActiveNode->hasToken = true;
                nodeToRemove->hasToken = false;
                APP_LOG_INFO("Token transferred from node {} to node {}", nodeId, nextActiveNode->getId());
            } else {
                token.reset();
                APP_LOG_INFO("Token invalidated");
            }
        }
        
        assignTokenRanges();
        repartitionData();

        nodeIdMap.erase(nodeId); // Remove from map

        if (simulationManager_) simulationManager_->onNodeRemoved(nodeId);
        nodes.erase(it);
        assignTokenRanges();

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
    spatialGrid.clear();
    for (const auto& node : nodes) {
        spatialGrid.insert(node.get());
    }

    if (!token || nodes.empty()) return;

    if (token->getTravelProgress() < 1.0f) {
        token->updateTravel(dt * 0.5f);
    }

    if (token->getTravelProgress() >= 1.0f) {
        int currentId = token->getCurrentNodeId();
        Node* currentNode = nullptr;
        
        // O(1) Lookup
        if (nodeIdMap.count(currentId)) currentNode = nodeIdMap[currentId];

        if (currentNode) {
            currentNode->hasToken = false;
            const auto& neighbors = currentNode->getNeighbors();
            int nextNodeId = -1;
            
            if (!neighbors.empty()) {
                int prevId = token->getPreviousNodeId();
                for (Node* n : neighbors) {
                    if (n->getId() != prevId) {
                        nextNodeId = n->getId();
                        break;
                    }
                }
                if (nextNodeId == -1) nextNodeId = neighbors[0]->getId();
            } else {
                nextNodeId = currentId;
            }

            token->moveToNextNode(nextNodeId);
            Node* nextNode = nullptr;
            
            // O(1) Lookup
            if (nodeIdMap.count(nextNodeId)) nextNode = nodeIdMap[nextNodeId];
            
            if (nextNode) nextNode->hasToken = true;
        }
    }
    
    processMessageQueue(dt);
    // resolveCollisions(); // Moved to updateNodeMovement
}

auto Ring::reorganizeNodes() -> void {
    // Disabled
}

auto Ring::findNodeIndexById(int nodeId) const -> int {
    // Still O(N) because we need Index, not Pointer
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i]->getId() == nodeId) return static_cast<int>(i);
    }
    return -1;
}

auto Ring::updateNodeMovement(float dt, Vector2 bounds) -> void {
    PROFILE_START("Physics_Movement");
    currentMaxVelocity = 0.0f;
    for (auto &node : nodes) {
        node->moveFreely(dt, bounds);
        float speed = Vector2Length(node->getVelocity());
        if (speed > currentMaxVelocity) currentMaxVelocity = speed;
    }
    
    PROFILE_START("Physics_Collisions");
    resolveCollisions();
    PROFILE_END("Physics_Collisions");
    
    PROFILE_END("Physics_Movement");
}

auto Ring::handleNodeDragging(Vector2 mousePos, bool mousePressed, const Camera2D &camera) -> void {
    Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);
    if (mousePressed) {
        // Optimization: Spatial Grid query instead of linear scan?
        // But visual picking is usually fine O(N) for interaction.
        for (auto &node : nodes) {
            if (Vector2Distance(worldPos, node->getPosition()) < 30.0f && !node->getDragging()) {
                node->setDragging(true);
                break;
            }
        }
    } else {
        for (auto &node : nodes) if (node->getDragging()) node->setDragging(false);
    }
    for (auto &node : nodes) if (node->getDragging()) node->setPosition(worldPos);
}

auto Ring::setAllNodesMobile(bool mobile) -> void {
    for (auto &node : nodes) {
        node->setMobile(mobile);
        if (mobile && node->getVelocity().x == 0 && node->getVelocity().y == 0) {
            node->setVelocity({(float)GetRandomValue(-50, 50), (float)GetRandomValue(-50, 50)});
        }
    }
}

auto Ring::shouldReorganize() const -> bool {
    for (const auto &node : nodes) if (!node->getMobile()) return true;
    return false;
}

auto Ring::reorganizeFromPositions() -> void {
    // Disabled for emergent
}

auto Ring::calculateRingCenter() -> Vector2 {
    if (nodes.empty()) return center;
    Vector2 avgPos = {0, 0};
    for (const auto &node : nodes) avgPos = Vector2Add(avgPos, node->getPosition());
    return Vector2Scale(avgPos, 1.0f / nodes.size());
}

auto Ring::resolveCollisions() -> void {
    float radius = 30.0f; 
    float diameter = radius * 2.0f;
    
    std::vector<Node*> neighbors; 
    neighbors.reserve(50);

    int iterations = (currentMaxVelocity < 5.0f) ? 1 : 4;

    for (int iter = 0; iter < iterations; ++iter) {
        for (auto& node : nodes) {
            if (!node->getMobile()) continue;
            Vector2 pos = node->getPosition();
            spatialGrid.query(pos, diameter, neighbors);
            for (Node* other : neighbors) {
                if (node.get() == other) continue;
                Vector2 otherPos = other->getPosition(); 
                float d = Vector2Distance(pos, otherPos);
                if (d < diameter) {
                    Vector2 dir;
                    if (d < 0.001f) {
                        d = 0.001f;
                        dir = { (float)GetRandomValue(-10, 10), (float)GetRandomValue(-10, 10) };
                        if (dir.x == 0 && dir.y == 0) dir.x = 1.0f;
                        dir = Vector2Normalize(dir);
                    } else {
                        dir = Vector2Subtract(pos, otherPos);
                        dir = Vector2Normalize(dir);
                    }
                    float overlap = diameter - d;
                    Vector2 push = Vector2Scale(dir, overlap * 0.5f);
                    node->setPosition(Vector2Add(pos, push));
                    other->setPosition(Vector2Subtract(otherPos, push));
                    pos = node->getPosition();
                }
            }
        }
    }
}

auto Ring::applyRingFormationForces() -> void {
    PROFILE_START("Physics_Forces");
    if (nodes.empty() || !ringFormationEnabled) { PROFILE_END("Physics_Forces"); return; }

    // Optimization: No nodeMap build needed! We have nodeIdMap or direct pointers.
    // But this function iterates 'nodes' anyway.
    // And 'bondedIds' are Node* pointers.
    // So we don't need map lookups.

    float breakDist = physics.searchRadius * 1.5f;
    float connectDist = physics.searchRadius;
    
    std::vector<Node*> candidates; 
    candidates.reserve(100);

    PROFILE_START("Forces_Pass1");
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        Vector2 pos = node->getPosition();

        // A. Maintenance
        std::vector<Node*> currentNeighbors = node->getNeighbors();
        for (Node* other : currentNeighbors) {
            if (Vector2Distance(pos, other->getPosition()) > breakDist) {
                APP_LOG_DEBUG("Bond broken (dist): {} <-> {}", node->getId(), other->getId());
                node->removeNeighbor(other);
                other->removeNeighbor(node.get()); 
            }
        }

        // B. Formation
        if (node->getNeighbors().size() < 2) {
            spatialGrid.query(pos, connectDist, candidates);
            std::vector<std::pair<float, Node*>> sorted;
            sorted.reserve(candidates.size());
            
            for (Node* other : candidates) {
                if (other == node.get()) continue;
                
                bool alreadyBonded = false;
                for(Node* n : node->getNeighbors()) if(n == other) alreadyBonded = true;
                if(alreadyBonded) continue;
                if (other->getNeighbors().size() >= 2) continue;

                // Check Size Limit (prevent forming giant rings)
                if (node->getClusterSize() > maxClusterSize || other->getClusterSize() > maxClusterSize) continue;
                if (node->getClusterId() != -1 && other->getClusterId() != -1 && node->getClusterId() != other->getClusterId()) {
                     if (node->getClusterSize() + other->getClusterSize() > maxClusterSize) continue;
                }

                float d = Vector2Distance(pos, other->getPosition());
                if (d < connectDist) sorted.push_back({d, other});
            }
            std::sort(sorted.begin(), sorted.end());

            for (const auto& item : sorted) {
                if (node->getNeighbors().size() >= 2) break;
                Node* other = item.second;
                if (other->getNeighbors().size() < 2) { 
                    APP_LOG_DEBUG("Bond formed: {} <-> {}", node->getId(), other->getId());
                    node->addNeighbor(other);
                    other->addNeighbor(node.get());
                }
            }
        }
    }
    PROFILE_END("Forces_Pass1");

    PROFILE_START("Forces_Pass2_BFS");
    std::unordered_map<int, int> clusterSizes;
    std::unordered_map<int, std::vector<Node*>> clusterEnds; 
    int nextClusterId = 0;
    for (auto& node : nodes) node->setClusterId(-1); 

    for (auto& node : nodes) {
        if (!node->getMobile()) continue;
        if (node->getClusterId() != -1) continue;

        int currentCluster = nextClusterId++;
        int size = 0;
        std::queue<Node*> q;
        node->setClusterId(currentCluster);
        q.push(node.get());
        size++;

        while(!q.empty()){
            Node* curr = q.front();
            q.pop();
            
            if (curr->getNeighbors().size() == 1) {
                clusterEnds[currentCluster].push_back(curr);
            }

            for (Node* neighbor : curr->getNeighbors()) {
                if (neighbor->getClusterId() == -1) {
                    neighbor->setClusterId(currentCluster);
                    q.push(neighbor);
                    size++;
                }
            }
        }
        clusterSizes[currentCluster] = size;
    }
    
    for(auto& node : nodes) {
        int cid = node->getClusterId();
        if(cid != -1) node->setClusterSize(clusterSizes[cid]);
        else node->setClusterSize(1);
    }
    PROFILE_END("Forces_Pass2_BFS");

    PROFILE_START("Forces_Pass3_Split");
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        int cid = node->getClusterId();
        int csize = (cid != -1) ? clusterSizes[cid] : 1;
        
        if (csize > maxClusterSize) {
            if (node->getNeighbors().size() >= 2) {
                if (GetRandomValue(0, 100) < 5) { 
                    Node* target = node->getNeighbors()[0];
                    APP_LOG_DEBUG("Bond broken (split): {} <-> {}", node->getId(), target->getId());
                    node->removeNeighbor(target);
                    target->removeNeighbor(node.get());
                }
            }
        }
        
        if (csize < maxClusterSize && node->getNeighbors().size() == 2) {
            Vector2 pos = node->getPosition();
            spatialGrid.query(pos, physics.idealDist * 0.6f, candidates);
            for (Node* intruder : candidates) {
                if (intruder == node.get()) continue;
                if (intruder->getNeighbors().size() < 2) {
                    // Ensure intruder is not already neighbor
                    bool isNeighbor = false;
                    for(Node* n : node->getNeighbors()) if(n == intruder) isNeighbor = true;
                    if(isNeighbor) continue;

                    Node* target = node->getNeighbors()[0];
                    node->removeNeighbor(target);
                    target->removeNeighbor(node.get());
                    break; 
                }
            }
        }
    }
    PROFILE_END("Forces_Pass3_Split");

    PROFILE_START("Forces_Pass4_Calc");
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        Vector2 pos = node->getPosition();
        Vector2 force = {0, 0};

        const auto& bonded = node->getNeighbors(); // Node*
        int myCluster = node->getClusterId();
        int myClusterSize = (myCluster != -1) ? clusterSizes[myCluster] : 1;

        // End-to-End
        if (bonded.size() == 1 && myCluster != -1) {
            const auto& ends = clusterEnds[myCluster];
            for (Node* endNode : ends) {
                if (endNode == node.get()) continue;
                Vector2 dir = Vector2Subtract(endNode->getPosition(), pos);
                float dist = Vector2Length(dir);
                if (dist > 0.1f) {
                    dir = Vector2Normalize(dir);
                    float pull = physics.chainAttractStrength * 5.0f; 
                    force = Vector2Add(force, Vector2Scale(dir, pull));
                }
            }
        }

        // Bonded Forces
        int neighborIndex = 0;
        for (Node* other : bonded) {
            if (myClusterSize > maxClusterSize && neighborIndex == 1) {
                Vector2 dir = Vector2Subtract(other->getPosition(), pos);
                dir = Vector2Normalize(dir);
                float repel = (physics.repulsionStrength * 50.0f);
                force = Vector2Subtract(force, Vector2Scale(dir, repel));
                neighborIndex++;
                continue; 
            }

            Vector2 dir = Vector2Subtract(other->getPosition(), pos);
            float dist = Vector2Length(dir);
            dir = Vector2Normalize(dir);

            float delta = dist - physics.idealDist;
            float k = (delta > 0) ? physics.chainAttractStrength : physics.repulsionStrength;
            force = Vector2Add(force, Vector2Scale(dir, delta * k));
            
            Vector2 tangent = {-dir.y, dir.x};
            force = Vector2Add(force, Vector2Scale(tangent, physics.vortexStrength));
            neighborIndex++;
        }

        // Non-Bonded
        spatialGrid.query(pos, physics.searchRadius, candidates);
        for (Node* other : candidates) {
            if (other == node.get()) continue;
            bool isBonded = false;
            for(Node* n : bonded) if(n == other) isBonded = true;
            if (isBonded) continue;

            float d = Vector2Distance(pos, other->getPosition());
            Vector2 dir = Vector2Subtract(other->getPosition(), pos);
            dir = Vector2Normalize(dir);

            int otherCluster = other->getClusterId();
            // otherSize is fast O(1) access
            int otherSize = (otherCluster != -1) ? other->getClusterSize() : 1;
            bool shouldMerge = false;
            
            if (myCluster != otherCluster) {
                if (myClusterSize + otherSize <= maxClusterSize) shouldMerge = true;
            }

            if (shouldMerge) {
                if (d < physics.searchRadius) {
                    force = Vector2Add(force, Vector2Scale(dir, physics.chainAttractStrength * 0.5f));
                }
            } else {
                if (d < physics.idealDist * 1.5f) { 
                    float repel = (physics.repulsionStrength * 50.0f) / (d + 1.0f);
                    force = Vector2Subtract(force, Vector2Scale(dir, repel));
                }
            }
        }

        node->applyForce(force);
        node->setVelocity(Vector2Scale(node->getVelocity(), physics.friction));
    }
    PROFILE_END("Forces_Pass4_Calc");
    PROFILE_END("Physics_Forces");
}

auto Ring::processMessageQueue(float dt) -> void {
    for (auto it = messageQueue.begin(); it != messageQueue.end(); ) {
        it->progress += dt * 2.0f;
        Node* start = nullptr; 
        Node* end = nullptr;
        // Optimization: Use Map
        if (nodeIdMap.count(it->currentNodeId)) start = nodeIdMap[it->currentNodeId];
        if (nodeIdMap.count(it->targetNodeId)) end = nodeIdMap[it->targetNodeId];

        if (!start || !end) {
            it = messageQueue.erase(it);
            continue;
        }
        it->startPos = start->getPosition();
        it->endPos = end->getPosition();
        if (it->progress >= 1.0f) {
            if (visualizer) {
                bool isReplica = it->content->isReplicationMessage;
                visualizer->startDataTransfer(it->startPos, it->endPos, it->content->data->getKey(), isReplica);
            }
            Node* targetNode = end;
            auto [accepted, forwardMsg] = targetNode->receiveMessage(std::move(*it->content));
            if (accepted) {
                // APP_LOG_INFO("Message delivered...");
                it = messageQueue.erase(it);
            } else {
                auto msg = std::move(forwardMsg);
                int currentId = targetNode->getId();
                it = messageQueue.erase(it); 
                routeMessage(currentId, std::move(msg)); 
            }
        } else {
            ++it;
        }
    }
}

// ... (findDataOwner, repartitionData etc can keep iterating vector if they are infrequent or O(N))
// But insertData should use map

auto Ring::findDataOwner(int hash) -> Node * {
    if (nodes.empty()) return nullptr;
    for (auto &node : nodes) {
        if (node->ownsHash(hash)) return node.get();
    }
    return nodes[0].get();
}

auto Ring::assignTokenRanges() -> void {
    if (nodes.empty()) return;
    int rangeSize = 360 / nodes.size();
    for (size_t i = 0; i < nodes.size(); ++i) {
        int start = i * rangeSize;
        int end = (i + 1) * rangeSize;
        if (i == nodes.size() - 1) end = 360;
        nodes[i]->setTokenRange(start, end);
    }
}

auto Ring::repartitionData() -> void {
    if (nodes.empty()) return;
    std::vector<std::unique_ptr<DataItem>> allData;
    for (auto &node : nodes) {
        auto &nodeData = node->storedData;
        while (!nodeData.empty()) {
            allData.push_back(std::move(nodeData.back()));
            nodeData.pop_back();
        }
    }
    assignTokenRanges();
    for (auto &data : allData) {
        int hash = data->getHash();
        Node *newOwner = findDataOwner(hash);
        if (newOwner) newOwner->addData(std::move(data));
    }
}

auto Ring::getDataDistribution() -> std::vector<int> {
    std::vector<int> distribution;
    for (const auto &node : nodes) distribution.push_back(node->getDataCount());
    return distribution;
}

auto Ring::forceRepartitionWithVisualization() -> void {
    if (nodes.empty() || !visualizer) return;
    for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& currentNode = nodes[i];
        for (const auto& data : currentNode->getStoredData()) {
            Node* correctOwner = findDataOwner(data->getHash());
            if (correctOwner && correctOwner != currentNode.get()) {
                visualizer->startDataTransfer(currentNode->getPosition(), correctOwner->getPosition(), data->getKey(), false);
            }
        }
    }
    repartitionData();
}

auto Ring::getSelectedNode() const -> Node* {
    if (selectedNodeId == -1) return nullptr;
    if (nodeIdMap.count(selectedNodeId)) return nodeIdMap.at(selectedNodeId);
    return nullptr;
}

auto Ring::getNextNode(int currentNodeId) -> Node* {
    Node* current = nullptr;
    if (nodeIdMap.count(currentNodeId)) current = nodeIdMap.at(currentNodeId);
    
    if (!current) return nullptr;
    
    const auto& neighbors = current->getNeighbors();
    if (neighbors.empty()) return current;
    // Neighbors are Node*
    return neighbors[0];
}

auto Ring::insertData(std::string key, std::string value) -> void {
    if (nodes.empty()) return;
    std::string keyCopy = key;
    std::string valueCopy = value;
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
    auto [accepted, forwardMsg] = startNode->receiveMessage(std::move(*msg));
    if (accepted) {
        APP_LOG_INFO("Data '{}' stored immediately at Node '{}'", keyCopy, startNode->getName());
    } else {
        routeMessage(startNode->getId(), std::move(forwardMsg));
    }
    int currentRF = replicationFactor;
    if (currentRF > nodes.size()) currentRF = static_cast<int>(nodes.size());
    Node* primaryOwner = findDataOwner(hash);
    if (!primaryOwner) {
        APP_LOG_ERROR("Primary owner node not found for hash: {}", hash);
        return;
    }
    
    // Use pointers logic
    Node* currentNode = primaryOwner;
    for (int i = 1; i < currentRF; ++i) {
        Node* nextNode = getNextNode(currentNode->getId());
        if (!nextNode || nextNode == currentNode) break;
        
        auto replicaData = std::make_unique<DataItem>(keyCopy, valueCopy, true);
        auto replicaMsg = std::make_unique<Node::RoutingMessage>();
        replicaMsg->data = std::move(replicaData);
        replicaMsg->targetHash = hash;
        replicaMsg->isReplicationMessage = true;
        replicaMsg->ttl = static_cast<int>(nodes.size()) * 2;
        
        routeMessage(currentNode->getId(), std::move(replicaMsg));
        APP_LOG_INFO("Replicating '{}' to Node '{}' (hop {})", keyCopy, nextNode->getName(), i);
        currentNode = nextNode;
    }
}

auto Ring::routeMessage(int startNodeId, std::unique_ptr<Node::RoutingMessage> msg) -> void {
    Node* current = nullptr;
    if (nodeIdMap.count(startNodeId)) current = nodeIdMap.at(startNodeId);
    
    if(!current) return;
    Node* next = getNextNode(startNodeId);
    if (next == current || !next) {
        APP_LOG_INFO("Node {} is isolated, cannot route message", startNodeId);
        return; 
    }
    PendingMessage pm;
    pm.content = std::move(msg);
    pm.currentNodeId = startNodeId;
    pm.targetNodeId = next->getId();
    pm.progress = 0.0f;
    pm.startPos = current->getPosition();
    pm.endPos = next->getPosition();
    messageQueue.push_back(std::move(pm));
}
