#include "Token.h"
#include <format>

Token::Token(int id) : Entity(id, "Token"), currentNodeId(0) {}

void Token::moveToNextNode(int nextNodeId) {
    previousNodeId = currentNodeId;
    currentNodeId = nextNodeId;
    travelProgress = 0.0f;
}

void Token::updateTravel(float deltaProgress) { travelProgress += deltaProgress; }

std::string Token::toString() const {
    return std::format("Token[id={}, currentNode={}, progress={:.2f}]", id, currentNodeId,
                       travelProgress);
}