#include "SimulationManager.h"
#include "utils/Logger.h"
#include <algorithm>
#include <fstream>
#include <format>

SimulationManager::SimulationManager() {
    APP_LOG_INFO("Simulation Manager initialized");
}

auto SimulationManager::addRing(Vector2 center, float radius) -> Ring& {
    auto ring = std::make_unique<Ring>(center, radius);
    ring->setRingId(nextRingId++); // Assign unique ID using setter
    if (visualizer) ring->setVisualizer(visualizer);
    
    // Set callback using lambda
    ring->setOnNodeRemovedCallback([this](int nodeId) {
        this->onNodeRemoved(nodeId);
    });

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
    PROFILE_START("Sim_Update");
    for (auto& ring : rings) {
        ring->update(dt);
        ring->updateNodeMovement(dt, {0,0}); // Infinite bounds
        ring->applyRingFormationForces(dt);
    }
    
    if (isBenchmarking) updateBenchmarkLogic(dt);
    
    PROFILE_END("Sim_Update");
    PROFILE_REPORT();
}

auto SimulationManager::clearSelection() -> void {
    // Deselect all currently selected nodes visually
    for (int nodeId : selectedNodes) {
        Node* node = findNodeById(nodeId);
        if (node) {
            node->setSelected(false);
        }
    }
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
    // Only select if NOT painting
    if (!isPainting && leftPressed) {
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

    // Painting Logic
    static int paintFrame = 0;
    if (isPainting && leftDown && !rings.empty()) {
        paintFrame++;
        if (paintFrame % 5 == 0) {
            auto& ring = *rings.front();
            ring.addNode(std::format("P_Node_{}", ring.getNodeCount()));
            Node& n = ring[ring.getNodeCount()-1];
            Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);
            n.setPosition(worldPos);
        }
    } else {
        paintFrame = 0;
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

auto SimulationManager::startBenchmark() -> void {
    isBenchmarking = true;
    benchStage = 0;
    benchTimer = 0.0f;
    benchSamples = 0;
    benchAccFPS = 0;
    benchMinFPS = 9999.0;
    benchMaxFPS = 0.0;
    benchAccPhys = 0;
    benchAccRender = 0;
    benchResults.clear();
    
    if (!rings.empty()) {
        auto& r = *rings.front();
        while(r.getNodeCount() > 0) r.removeLastNode();
    }
    
    SetTargetFPS(0); // Unlock FPS for benchmark
    APP_LOG_INFO("GRADUAL BENCHMARK STARTED");
}

auto SimulationManager::stopBenchmark() -> void {
    isBenchmarking = false;
    SetTargetFPS(60); // Restore cap
    
    std::ofstream f("benchmark_results.csv");
    f << "Nodes,AvgFPS,MinFPS,MaxFPS,Physics(ms),Render(ms)\n";
    for(const auto& d : benchResults) {
        f << std::format("{},{:.1f},{:.1f},{:.1f},{:.3f},{:.3f}\n", 
            d.nodeCount, d.fps, d.minFps, d.maxFps, d.physicsMs, d.renderMs);
    }
    f.close();
    
    APP_LOG_INFO("BENCHMARK COMPLETE. Results saved to benchmark_results.csv");
}

auto SimulationManager::getBenchmarkStatus() const -> std::string {
    if (!isBenchmarking) return "Idle";
    int nodes = 0;
    if (!rings.empty()) nodes = rings.front()->getNodeCount();
    return std::format("Stage {}/20 | Nodes: {}", benchStage, nodes);
}

auto SimulationManager::updateBenchmarkLogic(float dt) -> void {
    benchTimer += dt;
    
    // Phase 1: Warmup (0s - 3s) - Let physics settle
    if (benchTimer < 3.0f) return;
    
    // Phase 2: Measure (3s - 5s) - Accumulate stats
    if (benchTimer < 5.0f) {
        float currentFPS = (float)GetFPS();
        benchAccFPS += currentFPS;
        if (currentFPS < benchMinFPS) benchMinFPS = currentFPS;
        if (currentFPS > benchMaxFPS) benchMaxFPS = currentFPS;
        
        benchAccPhys += Profiler::instance().getAverageTime("Sim_Update");
        benchAccRender += Profiler::instance().getAverageTime("Vis_DrawRing");
        benchSamples++;
        return;
    }
    
    // Phase 3: Finalize Step (At 5s)
    if (!rings.empty()) {
        BenchmarkData data;
        data.nodeCount = rings.front()->getNodeCount();
        
        if (benchSamples > 0) {
            data.fps = (float)(benchAccFPS / benchSamples);
            data.minFps = (float)benchMinFPS;
            data.maxFps = (float)benchMaxFPS;
            data.physicsMs = benchAccPhys / benchSamples;
            data.renderMs = benchAccRender / benchSamples;
        } else {
            data.fps = 0; data.minFps = 0; data.maxFps = 0; 
            data.physicsMs = 0; data.renderMs = 0;
        }
        
        benchResults.push_back(data);
        
        APP_LOG_INFO("Bench Stage {}: {} nodes, {:.1f} avg FPS (Min: {:.1f})", benchStage, data.nodeCount, data.fps, data.minFps);
        
        if (benchStage >= 20 || data.fps < 10) {
            stopBenchmark();
            return;
        }
    }
    
    // Prepare Next Stage
    benchStage++;
    benchTimer = 0.0f;
    benchSamples = 0;
    benchAccFPS = 0;
    benchMinFPS = 9999.0;
    benchMaxFPS = 0.0;
    benchAccPhys = 0;
    benchAccRender = 0;
    
    if (rings.empty()) addRing({640,360}, 200);
    auto& r = *rings.front();
    for(int i=0; i<100; ++i) {
        r.addNode(std::format("Bench_{}_{}", benchStage, i));
        Node& n = r[r.getNodeCount()-1];
        n.setPosition({(float)GetRandomValue(100, 1180), (float)GetRandomValue(100, 620)});
    }
}
