#pragma once
#include "core/Node.h"
#include <vector>
#include <cmath>
#include <unordered_map>
#include <raymath.h>

// Simple spatial hashing for efficient neighbor queries
class SpatialGrid {
public:
    SpatialGrid(float cellSize) : cellSize(cellSize) {}

    void clear() {
        grid.clear();
    }

    void insert(Node* node) {
        int x = static_cast<int>(std::floor(node->getPosition().x / cellSize));
        int y = static_cast<int>(std::floor(node->getPosition().y / cellSize));
        int key = hash(x, y);
        grid[key].push_back(node);
    }

    // Query nodes within a 3x3 block of cells around the position
    // This is a rough approximation; for strict circle checks, filter results.
    std::vector<Node*> query(Vector2 position) {
        std::vector<Node*> results;
        int cx = static_cast<int>(std::floor(position.x / cellSize));
        int cy = static_cast<int>(std::floor(position.y / cellSize));

        for (int x = cx - 1; x <= cx + 1; ++x) {
            for (int y = cy - 1; y <= cy + 1; ++y) {
                int key = hash(x, y);
                auto it = grid.find(key);
                if (it != grid.end()) {
                    results.insert(results.end(), it->second.begin(), it->second.end());
                }
            }
        }
        return results;
    }

private:
    float cellSize;
    std::unordered_map<int, std::vector<Node*>> grid;

    // Simple hash function for 2D coordinates
    int hash(int x, int y) const {
        // Use a prime number based hash to reduce collisions
        // Assumes map coordinates won't exceed typical int ranges
        return x * 73856093 ^ y * 19349663;
    }
};
