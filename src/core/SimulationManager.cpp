#include "SimulationManager.h"
#include "utils/Logger.h"
#include <algorithm>

SimulationManager::SimulationManager() {
    SimLogInfo("Simulation Manager initialized");
}

auto SimulationManager::addRing(Vector2 center, float radius) -> Ring& {
    auto ring = std::make_unique<Ring>(center, radius);
    ring->setRingId(nextRingId++); // Assign unique ID using setter
    if (visualizer) ring->setVisualizer(visualizer);
    rings.push_back(std::move(ring));
    return *rings.back();
}

auto SimulationManager::removeRing(int index) -> void {
    if (index < 0 || index >= static_cast<int>(rings.size())) return;
    
    // Clear selection to avoid dangling pointers
    clearSelection();
    
    rings.erase(rings.begin() + index);
    SimLogInfo("Removed ring at index {}", index);
}

auto SimulationManager::update(float dt) -> void {
    for (auto& ring : rings) {
        ring->update(dt);
        ring->updateNodeMovement(dt, {0,0}); // Infinite bounds
        ring->applyRingFormationForces();
    }
}

auto SimulationManager::clearSelection() -> void {
    for (auto* node : selectedNodes) {
        node->setSelected(false);
    }
    selectedNodes.clear();
}

auto SimulationManager::selectNode(Node* node, bool multiSelect) -> void {
    if (!node) return;

    if (!multiSelect) {
        clearSelection();
    }

    // Toggle if already selected (and multi-select is on)
    if (multiSelect && node->getSelected()) {
        node->setSelected(false);
        std::erase(selectedNodes, node);
    } else if (!node->getSelected()) {
        node->setSelected(true);
        selectedNodes.push_back(node);
    }
}

auto SimulationManager::handleInput(const Camera2D& camera) -> void {
    Vector2 mousePos = GetMousePosition();
    Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);
    
    bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    bool leftDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

    // Handle Dragging logic (delegate to rings)
    for (auto& ring : rings) {
        ring->handleNodeDragging(mousePos, leftDown, camera);
    }

    // Handle Selection logic (Click)
    if (leftPressed) {
        Node* clickedNode = nullptr;
        
        // Check all nodes in all rings
        for (auto& ring : rings) {
            for (const auto& node : ring->getNodes()) {
                if (CheckCollisionPointCircle(worldPos, node->getPosition(), 30.0f)) {
                    clickedNode = node.get();
                    break; // Found one
                }
            }
            if (clickedNode) break;
        }

        if (clickedNode) {
            selectNode(clickedNode, shiftDown);
            SimLogInfo("Selected Node: {}", clickedNode->getName());
        } else if (!shiftDown) {
            // Clicked empty space -> clear selection
            clearSelection();
        }
    }
}
