#pragma once
#include "Entity.h"

class Ring;

class Token : public Entity {
private:
    int currentNodeId;
    int previousNodeId{-1};
    float travelProgress{0.0f};

public:
    explicit Token(int id);

    // Setters and Getters
    void setCurrentNodeId(int nodeId) { currentNodeId = nodeId; }
    int getCurrentNodeId() const { return currentNodeId; }
    int getPreviousNodeId() const { return previousNodeId; }

    // Movement Logic
    void moveToNextNode(int nextNodeId);
    void updateTravel(float deltaProgress);
    float getTravelProgress() const { return travelProgress; }

    std::string toString() const override;
};