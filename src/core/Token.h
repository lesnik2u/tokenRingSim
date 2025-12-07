#pragma once
#include "Entity.h"

class Ring; // forward declaration

class Token : public Entity {
private:
    int currentNodeId;
    int previousNodeId{-1}; // Track history for directional traversal
    float travelProgress{0.0f};
    
public:
    Token(int id);
    
    auto getCurrentNodeId() const -> int { return currentNodeId; }
    auto getPreviousNodeId() const -> int { return previousNodeId; }
    auto getTravelProgress() const -> float { return travelProgress; }
    
    auto moveToNextNode(int nextNodeId) -> void;
    auto updateTravel(float deltaProgress) -> void;
    
    auto toString() const -> std::string override;
    
    friend class Ring;
};
