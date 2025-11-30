#include "Ring.h"
#include <numbers>

Ring::Ring(Vector2 center, float radius) 
    : center(center), radius(radius) {}

auto Ring::addNode(std::string name) -> void {
    auto node = std::make_unique<Node>(nextNodeId++, std::move(name), 0.0f);
    nodes.push_back(std::move(node));
    reorganizeNodes();
}

auto Ring::removeLastNode() -> void {
    if (!nodes.empty()) {
        nodes.pop_back();
        reorganizeNodes();
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
    if (!token || nodes.empty()) return;
    
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
}

auto Ring::reorganizeNodes() -> void {
    if (nodes.empty()) return;
    
    float angleStep = (2.0f * std::numbers::pi_v<float>) / nodes.size();
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        nodes[i]->setAngle(i * angleStep);
        nodes[i]->updatePosition(center, radius);
    }
}
