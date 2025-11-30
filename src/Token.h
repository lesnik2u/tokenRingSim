#pragma once
#include "Entity.h"

class Ring; // forward declaration

class Token : public Entity {
private:
    int currentNodeId;
    float travelProgress{0.0f};
    
public:
    Token(int id);
    
    auto getCurrentNodeId() const -> int { return currentNodeId; }
    auto getTravelProgress() const -> float { return travelProgress; }
    
    auto moveToNextNode(int nextNodeId) -> void;
    auto updateTravel(float deltaProgress) -> void;
    
    auto toString() const -> std::string override;
    
    friend class Ring;
};
