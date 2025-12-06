#pragma once
#include <string>
#include <format>
#include <iostream>

class Entity {
protected:
    int id;
    std::string name;
    
public:
    Entity(int id, std::string name) : id(id), name(std::move(name)) {}
    virtual ~Entity() = default;
    
    [[nodiscard]] auto getId() const -> int { return id; }
    [[nodiscard]] auto getName() const -> std::string_view { return name; }
    
    virtual auto toString() const -> std::string {
        return std::format("Entity[id={}, name={}]", id, name);
    }
    
    friend auto operator<<(std::ostream& os, const Entity& e) -> std::ostream&;
};
