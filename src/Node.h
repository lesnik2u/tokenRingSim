#pragma once
#include "Entity.h"
#include <raylib.h>

class Ring;

class Node : public Entity {
private:
    Vector2 position;
    float angle;
    bool hasToken{false};
    Vector2 velocity{0, 0};
    bool isMobile{false};
    
public:
    Node(int id, std::string name, float angle);
    
    auto getPosition() const -> Vector2 { return position; }
    auto getAngle() const -> float { return angle; }
    auto hasTokenPresent() const -> bool { return hasToken; }
    
    auto setAngle(float newAngle) -> void;
    auto updatePosition(Vector2 center, float radius) -> void;

    auto setMobile(bool mobile) -> void { isMobile = mobile; }
    auto setVelocity(Vector2 vel) -> void { velocity = vel; }
    auto move(float dt, Vector2 bounds) -> void;
    
    auto toString() const -> std::string override;
    
    friend class Ring;
    friend auto operator==(const Node& lhs, const Node& rhs) -> bool;
};
