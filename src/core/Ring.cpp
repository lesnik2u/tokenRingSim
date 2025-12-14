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
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <limits>

Ring::Ring(Vector2 center, float radius) : center(center), radius(radius) {
    APP_LOG_INFO("Ring created at ({}, {}) with radius {}", center.x, center.y, radius);
}

Ring::Ring(const Ring &other)
    : center(other.center), radius(other.radius), nextNodeId(other.nextNodeId),
      replicationFactor(other.replicationFactor),
      ringFormationEnabled(other.ringFormationEnabled),
      visualizer(other.visualizer),
      ringId(other.ringId),
      onNodeRemovedCallback(other.onNodeRemovedCallback),
      spatialGrid(other.spatialGrid.getCellSize()), // Assuming constructor takes cell size
      currentMaxVelocity(other.currentMaxVelocity),
      timeSinceLastSteal(other.timeSinceLastSteal),
      sortingTimer(other.sortingTimer),
      sortingInterval(other.sortingInterval),
      physics(other.physics),
      maxClusterSize(other.maxClusterSize) {
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
    markTopologyDirty();
}

Ring::Ring(Ring &&other) noexcept
    : nodes(std::move(other.nodes)), token(std::move(other.token)), center(other.center),
      radius(other.radius), nextNodeId(other.nextNodeId), nodeIdMap(std::move(other.nodeIdMap)) {
          markTopologyDirty();
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
        markTopologyDirty();
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
        markTopologyDirty();
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
        markTopologyDirty();
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
        markTopologyDirty();
        
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

    // Handle Token
    if (token && node->getId() == token->getCurrentNodeId()) {
        Node* nextActiveNode = nullptr;
        if (nodes.size() > 1) {
            nextActiveNode = nodes[nodes.size() - 2].get(); // The new last node
        }
        
        if (nextActiveNode) {
            token->moveToNextNode(nextActiveNode->getId());
            nextActiveNode->hasToken = true;
            node->hasToken = false;
            APP_LOG_INFO("Token transferred from removed last node {} to node {}", node->getId(), nextActiveNode->getId());
        } else {
            token.reset();
            APP_LOG_INFO("Token invalidated (last node removed)");
        }
    }

    nodeIdMap.erase(node->getId());

    if (onNodeRemovedCallback) onNodeRemovedCallback(nodes.back()->getId());
    nodes.pop_back();
    markTopologyDirty();

    if (!nodes.empty()) {
        for(auto& d : orphans) nodes[0]->addData(std::move(d));
        repartitionData();
    }

    if (!token && !nodes.empty()) {
        spawnToken();
        APP_LOG_INFO("New token spawned after last node removal.");
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
        if (onNodeRemovedCallback) onNodeRemovedCallback(nodeId);
        nodes.erase(it);
        markTopologyDirty();

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
    sortingTimer += dt;
    
    if (sortingTimer >= sortingInterval) {
        reorganizeFromPositions();
        sortingTimer = 0.0f;
    }

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
        } else {
            // Token is stranded on a deleted node.
            APP_LOG_WARN("Token stranded on non-existent node {}. Respawning.", currentId);
            token.reset();
            spawnToken();
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

    // Always mark dirty to prevent stale grid data during collisions
    spatialGridDirty = true;

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

    for (auto& node : nodes) {
        int cid = node->getClusterId();
        clusterGroups[cid].push_back(node.get());
    }

    int sortedCount = 0;
    for (auto& [cid, cluster] : clusterGroups) {
        if (cluster.size() < 3) continue;

        // Calculate Centroid
        Vector2 centroid = {0, 0};
        for (Node* n : cluster) {
            centroid = Vector2Add(centroid, n->getPosition());
        }
        centroid = Vector2Scale(centroid, 1.0f / cluster.size());

        // Sort by Angle around Centroid
        std::sort(cluster.begin(), cluster.end(), [centroid](Node* a, Node* b) {
            float angA = std::atan2(a->getPosition().y - centroid.y, a->getPosition().x - centroid.x);
            float angB = std::atan2(b->getPosition().y - centroid.y, b->getPosition().x - centroid.x);
            return angA < angB;
        });

        // Re-Link
        float breakThreshold = physics.searchRadius * 1.1f; // Tighter threshold to prevent cross-map jumps
        for(Node* n : cluster) n->clearNeighbors();
        for(size_t i=0; i<cluster.size(); ++i) {
            Node* c = cluster[i];
            Node* n = cluster[(i+1) % cluster.size()];
            
            // Only connect if within physical limits.
            // This prevents the Sorter from creating "infinite" bonds that ignore Search Radius.
            if (Vector2Distance(c->getPosition(), n->getPosition()) < breakThreshold) {
                c->addNeighbor(n);
                n->addNeighbor(c);
            }
        }
        sortedCount++;
    }

    if (sortedCount > 0) {
        markTopologyDirty();
        repartitionData();
        // APP_LOG_INFO("Sorted {} disjoint ring(s) using Angular Sort.", sortedCount);
    }
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

    // Adaptive Iterations based on Kinetic Energy and Density
    int iterations = 1;
    if (currentMaxVelocity > 50.0f) iterations = 8;
    else if (currentMaxVelocity > 20.0f) iterations = 4;
    else if (currentMaxVelocity > 5.0f) iterations = 2;

    if (nodes.size() > 500) iterations += 2;
    if (iterations > 10) iterations = 10;

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
                        dir = Vector2Scale(dir, 1.0f / d);
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
    
        float breakDist = physics.idealDist * 2.5f;
        float connectDist = physics.idealDist * 1.2f;
        
        scratchBuffer.clear();
        std::vector<Node*>& candidates = scratchBuffer;
        if (candidates.capacity() < nodes.size()) candidates.reserve(nodes.size());
    
        PROFILE_START("Forces_Pass1");    // --- Pass 1: Maintenance & Basic Formation ---
    for (auto &node : nodes) {
        if (!node->getMobile()) continue;
        Vector2 pos = node->getPosition();

        // A. Maintenance
        std::vector<Node*> currentNeighbors = node->getNeighbors();
        for (Node* other : currentNeighbors) {
            float dist = Vector2Distance(pos, other->getPosition());
            bool shouldBreak = false;

            // Stress-based Age Logic
            // Only age the bond if it is stretched beyond comfort.
            // If it returns to a comfortable distance, reset age (grace period).
            if (dist > physics.idealDist * 1.1f) {
                node->incrementBondAge(other);
            } else {
                node->resetBondAge(other);
            }

            int age = node->getBondAge(other);
            if (dist > breakDist && age > 60) shouldBreak = true;
            if (dist > breakDist * 1.2f) shouldBreak = true; // Stricter maintenance

            if (shouldBreak) {
                APP_LOG_DEBUG("Bond broken (dist): {} <-> {}", node->getId(), other->getId());
                node->removeNeighbor(other);
                other->removeNeighbor(node.get());
                markClusterDirty(node->getClusterId());
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
                    markClusterDirty(node->getClusterId());
                    markClusterDirty(other->getClusterId());
                }
            }
        }
    }
    PROFILE_END("Forces_Pass1");

    // Replacement for global increment: Manage bond age based on stress
    // for (auto& node : nodes) node->incrementBondAges(); // REMOVED

    PROFILE_START("Forces_Pass2_BFS");
    if (topologyDirty) {
        // Full Rebuild
        clusterSizes.clear();
        clusterEnds.clear();
        dirtyClusters.clear();
        freeClusterIds.clear();
        globalNextClusterId = 0; // Reset ID counter for stability

        for (auto& node : nodes) node->setClusterId(-1);
        
        for (auto& node : nodes) {
            if (!node->getMobile()) continue;
            if (node->getClusterId() != -1) continue;

            int currentCluster = globalNextClusterId++;
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
        topologyDirty = false;
    } else if (!dirtyClusters.empty()) {
        // Incremental Update
        std::vector<Node*> affectedNodes;
        affectedNodes.reserve(nodes.size()); // Worst case
        
        // 1. Identify affected nodes and clear old stats
        for (auto& node : nodes) {
            int cid = node->getClusterId();
            if (dirtyClusters.count(cid)) {
                affectedNodes.push_back(node.get());
                node->setClusterId(-1);
            }
        }
        
        // Clear metadata for dirty clusters and recycle IDs
        for (int cid : dirtyClusters) {
            clusterSizes.erase(cid);
            clusterEnds.erase(cid);
            freeClusterIds.insert(cid); // Recycle this ID
        }
        dirtyClusters.clear();
        
        // 2. Re-run BFS on affected nodes
        for (Node* node : affectedNodes) {
            if (!node->getMobile()) continue;
            if (node->getClusterId() != -1) continue; // Already handled in this pass

            // Reuse ID if available
            int currentCluster;
            if (!freeClusterIds.empty()) {
                auto it = freeClusterIds.begin();
                currentCluster = *it;
                freeClusterIds.erase(it);
            } else {
                currentCluster = globalNextClusterId++;
            }

            int size = 0;
            std::queue<Node*> q;
            node->setClusterId(currentCluster);
            q.push(node);
            size++;

            while(!q.empty()){
                Node* curr = q.front();
                q.pop();

                if (curr->getNeighbors().size() == 1) {
                    clusterEnds[currentCluster].push_back(curr);
                }

                for (Node* neighbor : curr->getNeighbors()) {
                    // Only process neighbors that were also reset (part of the dirty set)
                    if (neighbor->getClusterId() == -1) {
                        neighbor->setClusterId(currentCluster);
                        q.push(neighbor);
                        size++;
                    }
                }
            }
            clusterSizes[currentCluster] = size;
        }
    }

    // Update individual node sizes (fast pass)
    for(auto& node : nodes) {
        int cid = node->getClusterId();
        if(cid != -1 && clusterSizes.count(cid)) node->setClusterSize(clusterSizes[cid]);
        else node->setClusterSize(1);
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
                    markClusterDirty(node->getClusterId());
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

                                if (intruder == node.get()) continue;
                                int mySize = (node->getClusterId() != -1) ? node->getClusterSize() : 1;
                                int otherSize = (intruder->getClusterId() != -1) ? intruder->getClusterSize() : 1;
                                
                                // Rule: Max-sized clusters are immune to theft
                                if (otherSize >= maxClusterSize) continue;

                                bool isIsolated = intruder->getNeighbors().empty();
                                bool canSteal = isIsolated;
                        
                                if (!isIsolated) {
                                    // Rule: Smaller rings cannot steal from bigger ones.
                                    // Hierarchy: Stealer must be strictly larger than victim.
                                    if (mySize > otherSize) {
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
                        
                                        markClusterDirty(node->getClusterId());
                                        markClusterDirty(intruder->getClusterId());
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
                
                // Explicit distance check to handle loose grid queries
                if (Vector2Distance(pos, other->getPosition()) > physics.searchRadius) continue;

                int myC = node->getClusterId();
                int otherC = other->getClusterId();
                if(myC == -1 || otherC == -1 || myC == otherC) continue;
                
                int totalSize = node->getClusterSize() + other->getClusterSize();
                if(totalSize > maxClusterSize) continue;
                
                // Increased probability (15%) to prevent starvation
                if(GetRandomValue(0, 100) > 15) continue; 
                
                Node* myBond = node->getNeighbors()[0];
                Node* otherBond = other->getNeighbors()[0];
                
                node->removeNeighbor(myBond);
                myBond->removeNeighbor(node.get());
                
                other->removeNeighbor(otherBond);
                otherBond->removeNeighbor(other);
                
                markClusterDirty(node->getClusterId());
                markClusterDirty(other->getClusterId());
                APP_LOG_INFO("Initiating merge of clusters (Size {} + {})", node->getClusterSize(), other->getClusterSize());
                goto EndMerge; 
            }
        }
    }
    EndMerge:;
    PROFILE_END("Forces_Pass3b_Merge");

    PROFILE_START("Forces_Pass3c_RingSwallow");
    // Attempt to merge disjoint closed rings (Swallow mechanics)
    if (GetRandomValue(0, 100) < 10) { // Throttle frequency
        for (auto& node : nodes) {
            if (!node->getMobile()) continue;
            if (node->getNeighbors().size() != 2) continue; // Must be in a ring (or middle of chain)

            int myC = node->getClusterId();
            int mySize = (myC != -1) ? node->getClusterSize() : 1;
            if (mySize >= maxClusterSize) continue; // I am already full

            Vector2 pos = node->getPosition();
            scratchBuffer.clear();
            spatialGrid.query(pos, physics.searchRadius * 0.8f, scratchBuffer); 

            for(Node* other : scratchBuffer) {
                if(other == node.get()) continue;
                if(other->getNeighbors().size() != 2) continue; // Target must also be in a ring

                // Explicit distance check
                if (Vector2Distance(pos, other->getPosition()) > physics.searchRadius * 0.8f) continue;

                int otherC = other->getClusterId();
                if(myC == -1 || otherC == -1 || myC == otherC) continue;

                int otherSize = (otherC != -1) ? other->getClusterSize() : 1;

                // Rule: Bigger swallows smaller (or equal). 
                // We strictly enforce mySize >= otherSize to define the predator.
                if (mySize < otherSize) continue; 

                // Rule: Cap Check
                if (mySize + otherSize > maxClusterSize) {
                    // Alternative: If we are too big to swallow whole, but we are dominantly larger,
                    // we "shatter" the smaller ring to make it vulnerable to piece-meal stealing.
                    if (mySize > otherSize && otherSize > 1) {
                         Node* victimBond = other->getNeighbors()[0];
                         other->removeNeighbor(victimBond);
                         victimBond->removeNeighbor(other);
                         markClusterDirty(other->getClusterId());
                         APP_LOG_INFO("Ring Shatter: Cluster {} (Size {}) shattered Cluster {} (Size {})", myC, mySize, otherC, otherSize);
                    }
                    continue;
                }

                // Perform Swallow (Bridge Merge)
                // Break a bond on both sides and stitch them together
                
                Node* bondA = node->getNeighbors()[0];
                Node* bondB = other->getNeighbors()[0];

                // 1. Break old bonds
                node->removeNeighbor(bondA);
                bondA->removeNeighbor(node.get());

                other->removeNeighbor(bondB);
                bondB->removeNeighbor(other);

                // 2. Form new cross bonds (The Bridge)
                node->addNeighbor(other);
                other->addNeighbor(node.get());

                bondA->addNeighbor(bondB);
                bondB->addNeighbor(bondA);

                markClusterDirty(node->getClusterId());
                markClusterDirty(other->getClusterId());
                APP_LOG_INFO("Ring Swallow: Cluster {} (Size {}) swallowed Cluster {} (Size {})", myC, mySize, otherC, otherSize);
                goto EndSwallow; 
            }
        }
    }
    EndSwallow:;
    PROFILE_END("Forces_Pass3c_RingSwallow");

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
            if (dist > 0.0001f) dir = Vector2Scale(dir, 1.0f / dist);
            else dir = {0, 0};

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

            Vector2 diff = Vector2Subtract(other->getPosition(), pos);
            float d = Vector2Length(diff);
            Vector2 dir;
            if (d > 0.0001f) dir = Vector2Scale(diff, 1.0f / d);
            else dir = {0, 0};

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
    int writeIdx = 0;
    for (int readIdx = 0; readIdx < static_cast<int>(messageQueue.size()); ++readIdx) {
        auto& pm = messageQueue[readIdx];
        pm.progress += dt * 2.0f;
        
        Node* start = nullptr;
        Node* end = nullptr;
        if (nodeIdMap.count(pm.currentNodeId)) start = nodeIdMap[pm.currentNodeId];
        if (nodeIdMap.count(pm.targetNodeId)) end = nodeIdMap[pm.targetNodeId];

        if (!start || !end) {
            // Drop message (node lost)
            continue;
        }
        
        pm.startPos = start->getPosition();
        pm.endPos = end->getPosition();

        if (pm.progress >= 1.0f) {
            if (visualizer) {
                bool isReplica = pm.content->isReplicationMessage;
                visualizer->startDataTransfer(pm.startPos, pm.endPos, pm.content->data->getKey(), isReplica);
            }
            Node* targetNode = end;
            auto [accepted, forwardMsg] = targetNode->receiveMessage(std::move(*pm.content));
            if (!accepted) {
                // If forwarded, it creates a NEW message via routeMessage
                auto msg = std::move(forwardMsg);
                int currentId = targetNode->getId();
                routeMessage(currentId, std::move(msg));
            }
            // Message consumed (don't write back)
        } else {
            // Keep message
            if (readIdx != writeIdx) {
                messageQueue[writeIdx] = std::move(pm);
            }
            writeIdx++;
        }
    }
    messageQueue.resize(writeIdx);
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

    // Use shared pointers to share string data across primary and replicas
    auto sharedKey = std::make_shared<std::string>(std::move(key));
    auto sharedValue = std::make_shared<std::string>(std::move(value));

    int startNodeIdx = GetRandomValue(0, static_cast<int>(nodes.size()) - 1);
    Node* startNode = nodes[startNodeIdx].get();
    
    auto data = std::make_unique<DataItem>(sharedKey, sharedValue);
    int hash = data->getHash();
    
    APP_LOG_INFO("Client request: Insert '{}' (hash={}°) -> Node '{}'", *sharedKey, hash, startNode->getName());
    auto msg = std::make_unique<Node::RoutingMessage>();
    msg->data = std::move(data);
    msg->targetHash = hash;
    msg->isReplicationMessage = false;
    msg->ttl = static_cast<int>(nodes.size()) * 2;
    auto [accepted, forwardMsg] = startNode->receiveMessage(std::move(*msg));
    if (accepted) {
        APP_LOG_INFO("Data '{}' stored immediately at Node '{}'", *sharedKey, startNode->getName());
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

        // Zero-copy string replication
        auto replicaData = std::make_unique<DataItem>(sharedKey, sharedValue, true);
        auto replicaMsg = std::make_unique<Node::RoutingMessage>();
        replicaMsg->data = std::move(replicaData);
        replicaMsg->targetHash = hash;
        replicaMsg->isReplicationMessage = true;
        replicaMsg->ttl = static_cast<int>(nodes.size()) * 2;

        routeMessage(currentNode->getId(), std::move(replicaMsg));
        APP_LOG_INFO("Replicating '{}' to Node '{}' (hop {})", *sharedKey, nextNode->getName(), i);
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
