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

    // Query nodes within the grid cells overlapping the given radius
    // Output results to the provided vector to avoid allocation overhead
    void query(Vector2 position, float radius, std::vector<Node*>& results) {
        results.clear();
        int minX = static_cast<int>(std::floor((position.x - radius) / cellSize));
        int maxX = static_cast<int>(std::floor((position.x + radius) / cellSize));
        int minY = static_cast<int>(std::floor((position.y - radius) / cellSize));
        int maxY = static_cast<int>(std::floor((position.y + radius) / cellSize));

        for (int x = minX; x <= maxX; ++x) {
            for (int y = minY; y <= maxY; ++y) {
                int key = hash(x, y);
                auto it = grid.find(key);
                if (it != grid.end()) {
                    results.insert(results.end(), it->second.begin(), it->second.end());
                }
            }
        }
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
