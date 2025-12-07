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
    if (other.token) {
        token = std::make_unique<Token>(other.token->getId());
        token->moveToNextNode(other.token->getCurrentNodeId());
        for (auto &node : nodes) {
            if (node->getId() == token->getCurrentNodeId()) {
                node->hasToken = true;
            }
        }
    }
}

Ring::Ring(Ring &&other) noexcept
    : nodes(std::move(other.nodes)), token(std::move(other.token)), center(other.center),
      radius(other.radius), nextNodeId(other.nextNodeId) {}

auto Ring::operator=(const Ring &other) -> Ring & {
    if (this != &other) {
        Ring temp(other);
        std::swap(nodes, temp.nodes);
        std::swap(token, temp.token);
        center = temp.center;
        radius = temp.radius;
        nextNodeId = temp.nextNodeId;
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
    nodes.push_back(std::move(node));
    assignTokenRanges();
    repartitionData();
}

auto Ring::removeLastNode() -> void {
    if (nodes.empty()) return;
    APP_LOG_DEBUG("Removing node: {}", nodes.back()->getName());
    repartitionData();
    if (simulationManager_) simulationManager_->onNodeRemoved(nodes.back()->getId());
    nodes.pop_back();
    assignTokenRanges();
}

auto Ring::removeNode(int nodeId) -> void {
    auto it = std::find_if(nodes.begin(), nodes.end(), [nodeId](const std::unique_ptr<Node>& n){
        return n->getId() == nodeId;
    });

    if (it != nodes.end()) {
        Node* nodeToRemove = it->get();
        messageQueue.erase(
            std::remove_if(messageQueue.begin(), messageQueue.end(),
                [nodeId](const PendingMessage& msg) {
                    return msg.currentNodeId == nodeId || msg.targetNodeId == nodeId;
                }),
            messageQueue.end()
        );
        
        if (token && nodeToRemove->getId() == token->getCurrentNodeId()) {
            size_t currentIdx = std::distance(nodes.begin(), it);
            if (nodes.size() > 1) {
                size_t nextIdx = (currentIdx + 1) % nodes.size();
                auto search_token_it = std::next(it);
                if (search_token_it == nodes.end()) search_token_it = nodes.begin();

                Node* nextActiveNode = nullptr;
                bool foundNextActive = false;
                auto start_search_token_it = search_token_it;
                do {
                    if (search_token_it->get()->isActive() && search_token_it->get()->getId() != nodeToRemove->getId()) {
                        nextActiveNode = search_token_it->get();
                        foundNextActive = true;
                        break;
                    }
                    std::advance(search_token_it, 1);
                    if (search_token_it == nodes.end()) search_token_it = nodes.begin();
                } while (search_token_it != start_search_token_it);

                if (foundNextActive && nextActiveNode) {
                    token->moveToNextNode(nextActiveNode->getId());
                    nextActiveNode->hasToken = true;
                    nodeToRemove->hasToken = false;
                    APP_LOG_INFO("Token transferred from node {} to node {}", nodeId, nextActiveNode->getId());
                } else {
                    token.reset();
                    APP_LOG_INFO("Token invalidated as no other active nodes found after node {}", nodeId);
                }
            } else {
                token.reset();
                APP_LOG_INFO("Token invalidated as the last node {} is removed", nodeId);
            }
        }
        
        assignTokenRanges();
        repartitionData();

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
        for(auto& n : nodes) {
            if(n->getId() == currentId) {
                currentNode = n.get();
                break;
            }
        }

        if (currentNode) {
            currentNode->hasToken = false;
            const auto& neighbors = currentNode->getNeighbors();
            int nextNodeId = -1;
            
            if (!neighbors.empty()) {
                int prevId = token->getPreviousNodeId();
                for (int nid : neighbors) {
                    if (nid != prevId) {
                        nextNodeId = nid;
                        break;
                    }
                }
                if (nextNodeId == -1) nextNodeId = neighbors[0];
            } else {
                nextNodeId = currentId;
            }

            token->moveToNextNode(nextNodeId);
            Node* nextNode = nullptr;
            for(auto& n : nodes) {
                if(n->getId() == nextNodeId) {
                    nextNode = n.get();
                    break;
                }
            }
            if (nextNode) nextNode->hasToken = true;
        }
    }
    
    processMessageQueue(dt);
}

auto Ring::reorganizeNodes() -> void {
    // Disabled
}

auto Ring::findNodeIndexById(int nodeId) const -> int {
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i]->getId() == nodeId) return static_cast<int>(i);
    }
    return -1;
}

auto Ring::updateNodeMovement(float dt, Vector2 bounds) -> void {
    for (auto &node : nodes) node->moveFreely(dt, bounds);
}

auto Ring::handleNodeDragging(Vector2 mousePos, bool mousePressed, const Camera2D &camera) -> void {
    Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);
    if (mousePressed) {
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
    float radius = 30.0f; // Visual radius
    float diameter = radius * 2.0f;
    
    // 4 Iterations for stiff collisions
    for (int iter = 0; iter < 4; ++iter) {
        for (auto& node : nodes) {
            if (!node->getMobile()) continue;
            Vector2 pos = node->getPosition();
            
            auto neighbors = spatialGrid.query(pos, diameter);
            
            for (Node* other : neighbors) {
                if (node.get() == other) continue;
                Vector2 otherPos = other->getPosition();
                float d = Vector2Distance(pos, otherPos);
                
                if (d < diameter && d > 0.001f) {
                    float overlap = diameter - d;
                    Vector2 dir = Vector2Subtract(pos, otherPos);
                    dir = Vector2Normalize(dir);
                    
                    Vector2 push = Vector2Scale(dir, overlap * 0.5f);
                    node->setPosition(Vector2Add(pos, push));
                }
            }
        }
    }
}

auto Ring::applyRingFormationForces() -> void {
    if (nodes.empty() || !ringFormationEnabled) return;

    // 1. Build Fast Lookup
    std::unordered_map<int, Node*> nodeMap;
    for (const auto& n : nodes) nodeMap[n->getId()] = n.get();

    float breakDist = physics.searchRadius * 1.5f;
    float connectDist = physics.searchRadius;

    // --- Pass 1: Maintenance & Basic Formation ---
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        Vector2 pos = node->getPosition();

        // A. Maintenance
        std::vector<int> currentNeighbors = node->getNeighbors();
        for (int id : currentNeighbors) {
            if (nodeMap.find(id) == nodeMap.end()) {
                node->removeNeighbor(id);
                continue;
            }
            Node* other = nodeMap[id];
            if (Vector2Distance(pos, other->getPosition()) > breakDist) {
                node->removeNeighbor(id);
                other->removeNeighbor(node->getId()); 
            }
        }

        // B. Formation
        if (node->getNeighbors().size() < 2) {
            auto candidates = spatialGrid.query(pos, connectDist);
            std::vector<std::pair<float, Node*>> sorted;
            for (Node* other : candidates) {
                if (other == node.get()) continue;
                
                bool alreadyBonded = false;
                for(int id : node->getNeighbors()) if(id == other->getId()) alreadyBonded = true;
                if(alreadyBonded) continue;
                if (other->getNeighbors().size() >= 2) continue;

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
                    node->addNeighbor(other->getId());
                    other->addNeighbor(node->getId());
                }
            }
        }
    }

    // --- Pass 2: Identify Clusters & Ends ---
    std::unordered_map<int, int> clusterSizes;
    std::unordered_map<int, std::vector<Node*>> clusterEnds; // For loop closing
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

            for (int neighborId : curr->getNeighbors()) {
                if (nodeMap.find(neighborId) != nodeMap.end()) {
                    Node* neighbor = nodeMap[neighborId];
                    if (neighbor->getClusterId() == -1) {
                        neighbor->setClusterId(currentCluster);
                        q.push(neighbor);
                        size++;
                    }
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

    // --- Pass 3: Topology Adjustments ---
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        int cid = node->getClusterId();
        int csize = (cid != -1) ? clusterSizes[cid] : 1;
        
        // Split if too big
        if (csize > maxClusterSize) {
            if (node->getNeighbors().size() >= 2) {
                if (GetRandomValue(0, 100) < 5) { 
                    int target = node->getNeighbors()[0];
                    node->removeNeighbor(target);
                    if (nodeMap.count(target)) nodeMap[target]->removeNeighbor(node->getId());
                }
            }
        }
        
        // Insert if room available
        if (csize < maxClusterSize && node->getNeighbors().size() == 2) {
            Vector2 pos = node->getPosition();
            auto candidates = spatialGrid.query(pos, physics.idealDist * 0.6f);
            for (Node* intruder : candidates) {
                if (intruder == node.get()) continue;
                if (intruder->getNeighbors().size() < 2) {
                    int target = node->getNeighbors()[0];
                    node->removeNeighbor(target);
                    if (nodeMap.count(target)) nodeMap[target]->removeNeighbor(node->getId());
                    break; 
                }
            }
        }
    }

    // --- Pass 4: Forces ---
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        Vector2 pos = node->getPosition();
        Vector2 force = {0, 0};

        const auto& bondedIds = node->getNeighbors();
        int myCluster = node->getClusterId();
        int myClusterSize = (myCluster != -1) ? clusterSizes[myCluster] : 1;

        // End-to-End Attraction (Close the Loop)
        if (bondedIds.size() == 1 && myCluster != -1) {
            const auto& ends = clusterEnds[myCluster];
            for (Node* endNode : ends) {
                if (endNode == node.get()) continue;
                Vector2 dir = Vector2Subtract(endNode->getPosition(), pos);
                float dist = Vector2Length(dir);
                if (dist > 0.1f) {
                    dir = Vector2Normalize(dir);
                    // Magnetic pull to close the ring
                    float pull = physics.chainAttractStrength * 5.0f; 
                    force = Vector2Add(force, Vector2Scale(dir, pull));
                }
            }
        }

        // Bonded Forces
        int neighborIndex = 0;
        for (int id : bondedIds) {
            if (nodeMap.find(id) == nodeMap.end()) continue;
            Node* other = nodeMap[id];
            
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

        // Non-Bonded Repulsion
        auto nearby = spatialGrid.query(pos, physics.searchRadius);
        for (Node* other : nearby) {
            if (other == node.get()) continue;
            bool isBonded = false;
            for(int id : bondedIds) if(id == other->getId()) isBonded = true;
            if (isBonded) continue;

            float d = Vector2Distance(pos, other->getPosition());
            Vector2 dir = Vector2Subtract(other->getPosition(), pos);
            dir = Vector2Normalize(dir);

            int otherCluster = other->getClusterId();
            int otherSize = (otherCluster != -1 && nodeMap.count(other->getId())) ? nodeMap[other->getId()]->getClusterSize() : 1;
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
}

auto Ring::processMessageQueue(float dt) -> void {
    for (auto it = messageQueue.begin(); it != messageQueue.end(); ) {
        it->progress += dt * 2.0f;
        Node* start = nullptr; 
        Node* end = nullptr;
        for(auto& n : nodes) {
            if(n->getId() == it->currentNodeId) start = n.get();
            if(n->getId() == it->targetNodeId) end = n.get();
        }
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
                APP_LOG_INFO("Message delivered to Node '{}'", targetNode->getName());
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
    for (const auto& node : nodes) {
        if (node->getId() == selectedNodeId) return node.get();
    }
    return nullptr;
}

auto Ring::getNextNode(int currentNodeId) -> Node* {
    Node* current = nullptr;
    for(auto& n : nodes) if(n->getId() == currentNodeId) current = n.get();
    
    if (!current) return nullptr;
    
    const auto& neighbors = current->getNeighbors();
    if (neighbors.empty()) return current;
    int nextId = neighbors[0];
    for(auto& n : nodes) if(n->getId() == nextId) return n.get();
    return nullptr;
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
    int primaryOwnerNodeId = primaryOwner->getId();
    int primaryOwnerIndex = findNodeIndexById(primaryOwnerNodeId);
    if (primaryOwnerIndex == -1) {
        APP_LOG_ERROR("Primary owner node not found for replication: {}", primaryOwnerNodeId);
        return;
    }
    Node* currentNode = nodes[primaryOwnerIndex].get();
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
    for(auto& n : nodes) if(n->getId() == startNodeId) current = n.get();
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