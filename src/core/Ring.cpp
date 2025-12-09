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
#include <unordered_set>
#include <limits>

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
    nodeIdMap.clear();
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
    topologyDirty = true;
}

Ring::Ring(Ring &&other) noexcept
    : nodes(std::move(other.nodes)), token(std::move(other.token)), center(other.center),
      radius(other.radius), nextNodeId(other.nextNodeId), nodeIdMap(std::move(other.nodeIdMap)) {
          topologyDirty = true;
}

auto Ring::operator=(const Ring &other) -> Ring & {
    if (this != &other) {
        Ring temp(other);
        std::swap(nodes, temp.nodes);
        std::swap(token, temp.token);
        center = temp.center;
        radius = temp.radius;
        nextNodeId = temp.nextNodeId;
        std::swap(nodeIdMap, temp.nodeIdMap);
        topologyDirty = true;
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
        topologyDirty = true;
    }
    return *this;
}

auto Ring::operator+=(std::string nodeName) -> Ring & {
    addNode(std::move(nodeName));
    return *this;
}

auto Ring::operator-=(std::string_view nodeName) -> Ring & {
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
        topologyDirty = true;
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

    Node* ptr = node.get();
    nodes.push_back(std::move(node));
    nodeIdMap[ptr->getId()] = ptr;
        topologyDirty = true;
        
        repartitionData();
        spatialGridDirty = true; 
    }
auto Ring::removeLastNode() -> void {
    if (nodes.empty()) return;
    Node* node = nodes.back().get();

    for (Node* neighbor : node->getNeighbors()) {
        neighbor->removeNeighbor(node);
    }

    APP_LOG_DEBUG("Removing node: {}", node->getName());

    // Save data
    std::vector<std::unique_ptr<DataItem>> orphans = node->clearData();

    nodeIdMap.erase(node->getId());

    if (simulationManager_) simulationManager_->onNodeRemoved(nodes.back()->getId());
    nodes.pop_back();
    topologyDirty = true;

    if (!nodes.empty()) {
        for(auto& d : orphans) nodes[0]->addData(std::move(d));
        repartitionData();
    }
}

auto Ring::removeNode(int nodeId) -> void {
    auto it = std::find_if(nodes.begin(), nodes.end(), [nodeId](const std::unique_ptr<Node>& n){
        return n->getId() == nodeId;
    });

    if (it != nodes.end()) {
        Node* nodeToRemove = it->get();

        for (Node* neighbor : nodeToRemove->getNeighbors()) {
            neighbor->removeNeighbor(nodeToRemove);
        }

        if (token && nodeToRemove->getId() == token->getCurrentNodeId()) {
            Node* nextActiveNode = nullptr;
            if (!nodeToRemove->getNeighbors().empty()) {
                nextActiveNode = nodeToRemove->getNeighbors()[0];
            }
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

        // Save data before deletion
        std::vector<std::unique_ptr<DataItem>> orphans = nodeToRemove->clearData();

        nodeIdMap.erase(nodeId);
        if (simulationManager_) simulationManager_->onNodeRemoved(nodeId);
        nodes.erase(it);
        topologyDirty = true;

        if (!nodes.empty()) {
            for(auto& d : orphans) nodes[0]->addData(std::move(d));
            repartitionData();
        }

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
    timeSinceLastSteal += dt;

    if (!token || nodes.empty()) return;

    if (token->getTravelProgress() < 1.0f) {
        token->updateTravel(dt * 0.5f);
    }

    if (token->getTravelProgress() >= 1.0f) {
        int currentId = token->getCurrentNodeId();
        Node* currentNode = nullptr;

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
            if (nodeIdMap.count(nextNodeId)) nextNode = nodeIdMap[nextNodeId];

            if (nextNode) nextNode->hasToken = true;
        }
    }

    processMessageQueue(dt);
}

auto Ring::reorganizeNodes() -> void {
    sortNodesAngularly();
}

auto Ring::updateNodeMovement(float dt, Vector2 bounds) -> void {
    PROFILE_START("Physics_Movement");
    currentMaxVelocity = 0.0f;
    for (auto &node : nodes) {
        node->moveFreely(dt, bounds);
        float speed = Vector2Length(node->getVelocity());
        if (speed > currentMaxVelocity) currentMaxVelocity = speed;
    }

    if (currentMaxVelocity > 0.1f) spatialGridDirty = true;

    if (spatialGridDirty) {
        spatialGrid.clear();
        for (const auto& node : nodes) {
            spatialGrid.insert(node.get());
        }
        spatialGridDirty = false;
    }

    PROFILE_START("Physics_Collisions");
    resolveCollisions();
    PROFILE_END("Physics_Collisions");

    PROFILE_END("Physics_Movement");
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
    for (auto &node : nodes) if (node->getDragging()) {
        node->setPosition(worldPos);
        spatialGridDirty = true;
    }
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
    sortNodesAngularly();
}

auto Ring::sortNodesAngularly() -> void {
    if (nodes.empty()) return;

    // 1. Group by Cluster ID (Reuse existing topology info)
    std::map<int, std::vector<Node*>> clusterGroups;
    bool anyValidCluster = false;

    for (auto& node : nodes) {
        int cid = node->getClusterId();
        clusterGroups[cid].push_back(node.get());
    }

    int sortedCount = 0;
    for (auto& [cid, cluster] : clusterGroups) {
        if (cluster.size() < 3) continue;

        // Sort by Nearest Neighbor (TSP Heuristic)
        std::vector<Node*> sorted;
        sorted.reserve(cluster.size());
        std::unordered_set<Node*> used;
        std::unordered_set<Node*> clusterSet(cluster.begin(), cluster.end());

        Node* curr = cluster[0];
        sorted.push_back(curr);
        used.insert(curr);

        std::vector<Node*> gridResults;
        gridResults.reserve(50);

        while(sorted.size() < cluster.size()) {
            float minDist = std::numeric_limits<float>::max();
            Node* best = nullptr;

            // Optimization: Try Spatial Grid first
            spatialGrid.query(curr->getPosition(), 500.0f, gridResults);

            for(Node* candidate : gridResults) {
                if(used.count(candidate) || !clusterSet.count(candidate)) continue;
                float d = Vector2Distance(curr->getPosition(), candidate->getPosition());
                if (d < minDist) {
                    minDist = d;
                    best = candidate;
                }
            }

            // Fallback to linear scan if grid failed
            if (!best) {
                for(Node* candidate : cluster) {
                    if(used.count(candidate)) continue;
                    float d = Vector2Distance(curr->getPosition(), candidate->getPosition());
                    if (d < minDist) {
                        minDist = d;
                        best = candidate;
                    }
                }
            }

            if (best) {
                curr = best;
                sorted.push_back(curr);
                used.insert(curr);
            } else {
                break;
            }
        }

        // Re-Link
        for(Node* n : sorted) n->clearNeighbors();
        for(size_t i=0; i<sorted.size(); ++i) {
            Node* c = sorted[i];
            Node* n = sorted[(i+1) % sorted.size()];
            c->addNeighbor(n);
            n->addNeighbor(c);
        }
        sortedCount++;
    }

    topologyDirty = true;
    repartitionData();
    APP_LOG_INFO("Sorted {} disjoint ring(s) using Nearest Neighbor.", sortedCount);
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

auto Ring::applyRingFormationForces(float dt) -> void {
    PROFILE_START("Physics_Forces");
        if (nodes.empty() || !ringFormationEnabled) { PROFILE_END("Physics_Forces"); return; }
    
        float breakDist = physics.searchRadius * 2.0f;
        float connectDist = physics.searchRadius * 0.8f;
        
        scratchBuffer.clear();
        std::vector<Node*>& candidates = scratchBuffer;
        candidates.reserve(100);
    
        PROFILE_START("Forces_Pass1");    // --- Pass 1: Maintenance & Basic Formation ---
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        Vector2 pos = node->getPosition();

        // A. Maintenance
        std::vector<Node*> currentNeighbors = node->getNeighbors();
        for (Node* other : currentNeighbors) {
            float dist = Vector2Distance(pos, other->getPosition());
            bool shouldBreak = false;

            int age = node->getBondAge(other);
            if (dist > breakDist && age > 60) shouldBreak = true;
            if (dist > breakDist * 2.0f) shouldBreak = true;

            if (shouldBreak) {
                APP_LOG_DEBUG("Bond broken (dist): {} <-> {}", node->getId(), other->getId());
                node->removeNeighbor(other);
                other->removeNeighbor(node.get());
                topologyDirty = true;
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

                // Deterministic
                if (node->getId() > other->getId()) continue;

                // Future Size Check
                int mySize = (node->getClusterId() != -1) ? node->getClusterSize() : 1;
                int otherSize = (other->getClusterId() != -1) ? other->getClusterSize() : 1;
                int combinedSize = (node->getClusterId() == other->getClusterId()) ? mySize : mySize + otherSize;

                if (node->getClusterId() != other->getClusterId()) {
                    if (combinedSize > maxClusterSize) continue;
                }
                if (mySize > maxClusterSize) continue;

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
                    topologyDirty = true;
                }
            }
        }
    }
    PROFILE_END("Forces_Pass1");

    for (auto& node : nodes) node->incrementBondAges();

    PROFILE_START("Forces_Pass2_BFS");
    if (topologyDirty) {
        clusterSizes.clear();
        clusterEnds.clear();
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
        topologyDirty = false;
    }
    PROFILE_END("Forces_Pass2_BFS");

    PROFILE_START("Forces_Pass3_Split");
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        int cid = node->getClusterId();
        int csize = (cid != -1) ? clusterSizes[cid] : 1;

        // Split
        if (csize > maxClusterSize) {
            if (node->getNeighbors().size() >= 2) {
                if (GetRandomValue(0, 100) < 10) {
                    Node* target = node->getNeighbors()[0];
                    APP_LOG_DEBUG("Bond broken (split): {} <-> {}", node->getId(), target->getId());
                    node->removeNeighbor(target);
                    target->removeNeighbor(node.get());
                    topologyDirty = true;
                }
            }
        }

                        // Insert (Predatory)

                        if (csize < maxClusterSize && node->getNeighbors().size() == 2 && timeSinceLastSteal > 2.0f) {

                            

                            if (GetRandomValue(0, 100) >= 2) continue; 

                

                            Vector2 pos = node->getPosition();

                            // Moderate radius: 1.4x ideal. Allows gradual pull before snap.

                            spatialGrid.query(pos, physics.idealDist * 1.4f, candidates);

                            for (Node* intruder : candidates) {

                                if (intruder == node.get()) continue;                        int mySize = (node->getClusterId() != -1) ? node->getClusterSize() : 1;
                        int otherSize = (intruder->getClusterId() != -1) ? intruder->getClusterSize() : 1;
                        bool isIsolated = intruder->getNeighbors().empty();
                        bool canSteal = isIsolated;
                        
                        if (!isIsolated) {
                            if (mySize >= otherSize * 3 && otherSize <= 2) {
                                if (node->getClusterId() != intruder->getClusterId()) canSteal = true;
                            }
                        }
                        
                        if (!canSteal) continue;
        
                                        // Smart Target Selection: Break bond with neighbor closest to intruder
                                        Node* n1 = node->getNeighbors()[0];
                                        Node* n2 = node->getNeighbors()[1];
                                        float d1 = Vector2Distance(intruder->getPosition(), n1->getPosition());
                                        float d2 = Vector2Distance(intruder->getPosition(), n2->getPosition());
                                        Node* target = (d1 < d2) ? n1 : n2;
                        
                                        bool isStillNeighbor = false;
                                        for(Node* n : node->getNeighbors()) if(n == target) isStillNeighbor = true;
                                        if(!isStillNeighbor) continue;
                        
                                        if (!isIsolated) {
                                             std::vector<Node*> oldNeighbors = intruder->getNeighbors();
                                             for(Node* n : oldNeighbors) {
                                                 intruder->removeNeighbor(n);
                                                 n->removeNeighbor(intruder);
                                             }
                                             APP_LOG_DEBUG("Stole node {} from smaller ring", intruder->getId());
                                        }
                        
                                        APP_LOG_DEBUG("Inserting node {} into ring (via {})", intruder->getId(), node->getId());
                                        
                                        // 1. Break old bond
                                        node->removeNeighbor(target);
                                        target->removeNeighbor(node.get());
                                        
                                        // 2. Form new bonds immediately (Stitch the gap)
                                        node->addNeighbor(intruder);
                                        intruder->addNeighbor(node.get());
                                        
                                        target->addNeighbor(intruder);
                                        intruder->addNeighbor(target);
                        
                                        topologyDirty = true;
                                        timeSinceLastSteal = 0.0f;
                                        break;                    }
                }    }
    PROFILE_END("Forces_Pass3_Split");

    PROFILE_START("Forces_Pass3b_Merge");
    if (currentMaxVelocity < 20.0f) {
        for (auto& node : nodes) {
            if (node->getNeighbors().size() < 2) continue; 
            
            Vector2 pos = node->getPosition();
            scratchBuffer.clear();
            spatialGrid.query(pos, physics.searchRadius, scratchBuffer);
            
            for(Node* other : scratchBuffer) {
                if(other == node.get()) continue;
                if(other->getNeighbors().size() < 2) continue;
                
                int myC = node->getClusterId();
                int otherC = other->getClusterId();
                if(myC == -1 || otherC == -1 || myC == otherC) continue;
                
                int totalSize = node->getClusterSize() + other->getClusterSize();
                if(totalSize > maxClusterSize) continue;
                
                if(GetRandomValue(0, 100) > 1) continue; 
                
                Node* myBond = node->getNeighbors()[0];
                Node* otherBond = other->getNeighbors()[0];
                
                node->removeNeighbor(myBond);
                myBond->removeNeighbor(node.get());
                
                other->removeNeighbor(otherBond);
                otherBond->removeNeighbor(other);
                
                topologyDirty = true;
                APP_LOG_INFO("Initiating merge of clusters (Size {} + {})", node->getClusterSize(), other->getClusterSize());
                goto EndMerge; 
            }
        }
    }
    EndMerge:;
    PROFILE_END("Forces_Pass3b_Merge");

    PROFILE_START("Forces_Pass4_Calc");
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        Vector2 pos = node->getPosition();
        Vector2 force = {0, 0};

        const auto& bondedIds = node->getNeighbors();
        int myCluster = node->getClusterId();

        // End-to-End
        if (bondedIds.size() == 1 && myCluster != -1) {
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
        for (Node* other : bondedIds) {
            Vector2 dir = Vector2Subtract(other->getPosition(), pos);
            float dist = Vector2Length(dir);
            dir = Vector2Normalize(dir);

            float delta = dist - physics.idealDist;
            float k = (delta > 0) ? physics.chainAttractStrength : physics.repulsionStrength;
            force = Vector2Add(force, Vector2Scale(dir, delta * k));

            Vector2 tangent = {-dir.y, dir.x};
            force = Vector2Add(force, Vector2Scale(tangent, physics.vortexStrength));
        }

        // Non-Bonded
        spatialGrid.query(pos, physics.searchRadius, candidates);
        for (Node* other : candidates) {
            if (other == node.get()) continue;
            bool isBonded = false;
            for(Node* n : bondedIds) if(n == other) isBonded = true;
            if (isBonded) continue;

            float d = Vector2Distance(pos, other->getPosition());
            Vector2 dir = Vector2Subtract(other->getPosition(), pos);
            dir = Vector2Normalize(dir);

            int otherCluster = other->getClusterId();
            int otherSize = (otherCluster != -1) ? other->getClusterSize() : 1;
            bool shouldMerge = false;

            if (myCluster != otherCluster) {
                if (node->getClusterSize() + otherSize <= maxClusterSize) shouldMerge = true;
            }

            if (shouldMerge) {
                if (d < physics.searchRadius) {
                    // Strong attraction to ensure merge happens
                    force = Vector2Add(force, Vector2Scale(dir, physics.chainAttractStrength * 5.0f));
                }
            } else {
                if (d < physics.idealDist * 1.5f) {
                    float repel = (physics.repulsionStrength * 50.0f) / (d + 1.0f);
                    force = Vector2Subtract(force, Vector2Scale(dir, repel));
                }
            }
        }

        node->applyForce(force, dt);
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

    // Sort temporary list by physical angle to align token ranges with visual position
    std::vector<Node*> sortedNodes;
    sortedNodes.reserve(nodes.size());
    for(const auto& n : nodes) sortedNodes.push_back(n.get());

    Vector2 c = calculateRingCenter();
    std::sort(sortedNodes.begin(), sortedNodes.end(), [c](Node* a, Node* b) {
        float angA = std::atan2(a->getPosition().y - c.y, a->getPosition().x - c.x);
        float angB = std::atan2(b->getPosition().y - c.y, b->getPosition().x - c.x);
        return angA < angB;
    });

    int rangeSize = 360 / nodes.size();
    for (size_t i = 0; i < sortedNodes.size(); ++i) {
        int start = i * rangeSize;
        int end = (i + 1) * rangeSize;
        if (i == sortedNodes.size() - 1) end = 360;
        sortedNodes[i]->setTokenRange(start, end);
    }
}

auto Ring::repartitionData() -> void {
    if (nodes.empty()) return;
    std::vector<std::unique_ptr<DataItem>> allData;
    for (auto &node : nodes) {
        auto extracted = node->clearData();
        for(auto& item : extracted) allData.push_back(std::move(item));
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
    int nextId = neighbors[0]->getId();
    if (nodeIdMap.count(nextId)) return nodeIdMap.at(nextId);
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
    Node* currentNode = nodeIdMap.count(primaryOwnerNodeId) ? nodeIdMap.at(primaryOwnerNodeId) : nullptr;

    if (!currentNode) {
        APP_LOG_ERROR("Primary owner node not found in map: {}", primaryOwnerNodeId);
        return;
    }

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
