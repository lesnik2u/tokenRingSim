#pragma once
#include <vector>
#include <memory>
#include "Node.h"
#include "Token.h"
#include <raylib.h>

class Ring {
private:
    std::vector<std::unique_ptr<Node>> nodes;
    std::unique_ptr<Token> token;
    Vector2 center;
    float radius;
    int nextNodeId{0};
    
    auto reorganizeNodes() -> void;
    
public:
    Ring(Vector2 center, float radius);
    
    auto addNode(std::string name) -> void;
    auto removeLastNode() -> void;
    auto spawnToken() -> void;
    auto update(float dt) -> void;
    
    auto getNodes() const -> const std::vector<std::unique_ptr<Node>>& { return nodes; }
    auto getToken() const -> const Token* { return token.get(); }
    auto getCenter() const -> Vector2 { return center; }
    auto getRadius() const -> float { return radius; }
    auto getNodeCount() const -> size_t { return nodes.size(); }
    
    friend class Visualizer;
};
