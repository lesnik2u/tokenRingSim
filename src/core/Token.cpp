#include "Token.h"
#include <format>

Token::Token(int id)
    : Entity(id, "Token"), currentNodeId(0) {
}
auto Token::moveToNextNode(int nextNodeId) -> void {
    previousNodeId = currentNodeId;
    currentNodeId = nextNodeId;
    travelProgress = 0.0f;
}

auto Token::updateTravel(float deltaProgress) -> void {
    travelProgress += deltaProgress;
}

auto Token::toString() const -> std::string {
    return std::format("Token[id={}, currentNode={}, progress={:.2f}]",
                       id, currentNodeId, travelProgress);
}
