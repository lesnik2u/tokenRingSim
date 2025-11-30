#include "Node.h"
#include <cmath>
#include <format>

Node::Node(int id, std::string name, float angle) 
    : Entity(id, std::move(name)), angle(angle), position{0, 0} {}

auto Node::setAngle(float newAngle) -> void {
    angle = newAngle;
}

auto Node::updatePosition(Vector2 center, float radius) -> void {
    position.x = center.x + radius * std::cos(angle);
    position.y = center.y + radius * std::sin(angle);
}

auto Node::toString() const -> std::string {
    return std::format("Node[id={}, name={}, angle={:.2f}, hasToken={}]", 
                       id, name, angle, hasToken);
}

auto operator==(const Node& lhs, const Node& rhs) -> bool {
    return lhs.id == rhs.id;
}
