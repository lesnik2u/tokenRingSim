#pragma once
#include <format>
#include <iostream>
#include <string>

class Entity {
protected:
    int id;
    std::string name;

public:
    // Constructor
    Entity(int id, std::string name) : id(id), name(std::move(name)) {}

    // Virtual destructor
    virtual ~Entity() = default;

    // Getters
    int getId() const { return id; }
    std::string_view getName() const { return name; }

    // String representation
    virtual std::string toString() const { return std::format("Entity[id={}, name={}]", id, name); }

    friend std::ostream &operator<<(std::ostream &os, const Entity &e);
};
