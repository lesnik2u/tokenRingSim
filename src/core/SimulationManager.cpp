#include "SimulationManager.h"
#include "utils/Logger.h"
#include <algorithm>

SimulationManager::SimulationManager() {
    APP_LOG_INFO("Simulation Manager initialized");
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
    APP_LOG_INFO("Removed ring at index {}", index);
}

auto SimulationManager::update(float dt) -> void {
    for (auto& ring : rings) {
        ring->update(dt);
        ring->updateNodeMovement(dt, {0,0}); // Infinite bounds
        ring->applyRingFormationForces();
    }
}

auto SimulationManager::clearSelection() -> void {
    selectedNodes.clear();
}

auto SimulationManager::selectNode(int nodeId, bool multiSelect) -> void {
    Node* node = findNodeById(nodeId);
    if (!node) return;

    if (!multiSelect) {
        // Deselect all previously selected nodes
        for (int previouslySelectedId : selectedNodes) {
            Node* prevNode = findNodeById(previouslySelectedId);
            if (prevNode) {
                prevNode->setSelected(false);
            }
        }
        clearSelection(); // Clear the IDs as well
    }

    auto it = std::find(selectedNodes.begin(), selectedNodes.end(), nodeId);

    if (it != selectedNodes.end()) { // Node is currently selected
        node->setSelected(false);
        selectedNodes.erase(it);
    } else { // Node is not currently selected
        node->setSelected(true);
        selectedNodes.push_back(nodeId);
    }
}

auto SimulationManager::handleInput(const Camera2D& camera) -> void {
    Vector2 mousePos = GetMousePosition();
    bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    bool leftDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

    // Handle Dragging logic (delegate to rings)
    for (auto& ring : rings) {
        ring->handleNodeDragging(mousePos, leftDown, camera);
    }

    // Handle Node selection (click) via Visualizer
    if (leftPressed) {
        int clickedNodeId = -1;
        // Iterate through all rings and ask the visualizer to check for clicks
        if (visualizer) {
            for (auto& ring : rings) {
                clickedNodeId = visualizer->checkNodeClick(*ring);
                if (clickedNodeId != -1) {
                    // A node was clicked in this ring
                    break;
                }
            }
        }

        if (clickedNodeId != -1) {
            Node* clickedNode = findNodeById(clickedNodeId);
            if (clickedNode) {
                selectNode(clickedNodeId, shiftDown);
                APP_LOG_INFO("Selected Node: {}", clickedNode->getName());
            }
        } else if (!shiftDown) {
            // Clicked empty space -> clear selection if not multi-selecting
            clearSelection();
        }
    }
}

auto SimulationManager::findNodeById(int nodeId) const -> Node* {
    for (const auto& ring : rings) {
        for (const auto& node : ring->getNodes()) {
            if (node->getId() == nodeId) {
                return node.get();
            }
        }
    }
    return nullptr;
}

auto SimulationManager::onNodeRemoved(int nodeId) -> void {
    auto it = std::remove(selectedNodes.begin(), selectedNodes.end(), nodeId);
    selectedNodes.erase(it, selectedNodes.end());

    // Ensure the node's selected state is false, in case it was selected
    Node* removedNode = findNodeById(nodeId);
    if (removedNode) { // It might have been already deallocated if not selected
        removedNode->setSelected(false);
    }
}

