#pragma once
#include "core/Node.h"
#include <cmath>
#include <raymath.h>
#include <unordered_map>
#include <vector>

class SpatialGrid {
public:
    SpatialGrid(float cellSize) : cellSize(cellSize) {}

    // Resets the grid content
    void clear() {
        if (++clearCounter > 100) {
            grid.clear();
            clearCounter = 0;
        } else {
            for (auto &kv : grid) {
                kv.second.clear();
            }
        }
    }

    // Inserts a node into its corresponding cell
    void insert(Node *node) {
        int x = static_cast<int>(std::floor(node->getPosition().x / cellSize));
        int y = static_cast<int>(std::floor(node->getPosition().y / cellSize));
        int key = hash(x, y);
        grid[key].push_back(node);
    }

    // Finds all nodes within a certain distance
    void query(Vector2 position, float radius, std::vector<Node *> &results) const {
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
                    const std::vector<Node *> &cell = it->second;
                    if (!cell.empty()) {
                        results.insert(results.end(), cell.begin(), cell.end());
                    }
                }
            }
        }
    }

private:
    float cellSize;
    std::unordered_map<int, std::vector<Node *>> grid;
    int clearCounter = 0;

    // Spatial hash function
    int hash(int x, int y) const { return x * 73856093 ^ y * 19349663; }

public:
    float getCellSize() const { return cellSize; }
};