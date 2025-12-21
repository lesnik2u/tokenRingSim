#include "SimulationManager.h"
#include "utils/Logger.h"
#include <algorithm>
#include <format>
#include <fstream>

SimulationManager::SimulationManager() { APP_LOG_INFO("Simulation Manager initialized"); }

Ring &SimulationManager::addRing(Vector2 center, float radius) {
    auto ring = std::make_unique<Ring>(center, radius);
    ring->setRingId(nextRingId++);
    if (visualizer)
        ring->setVisualizer(visualizer);

    ring->setOnNodeRemovedCallback([this](int nodeId) { this->onNodeRemoved(nodeId); });

    ring->setOnNodeAddedCallback([this](Node *node) {
        if (node)
            this->globalNodeMap[node->getId()] = node;
    });

    rings.push_back(std::move(ring));

    for (const auto &node : rings.back()->getNodes()) {
        globalNodeMap[node->getId()] = node.get();
    }

    return *rings.back();
}

void SimulationManager::removeRing(int index) {
    if (index < 0 || index >= static_cast<int>(rings.size()))
        return;

    clearSelection();

    for (const auto &node : rings[index]->getNodes()) {
        globalNodeMap.erase(node->getId());
    }

    rings.erase(rings.begin() + index);
    APP_LOG_INFO("Removed ring at index {}", index);
}

void SimulationManager::update(float dt) {
    PROFILE_START("Sim_Update");

    Vector2 screenBounds = {(float)GetScreenWidth(), (float)GetScreenHeight()};
    for (auto &ring : rings) {
        ring->update(dt);
        ring->updateNodeMovement(dt, screenBounds);
        ring->applyRingFormationForces(dt);
    }

    if (isBenchmarking)
        updateBenchmarkLogic(dt);

    PROFILE_END("Sim_Update");
    PROFILE_REPORT();
}

void SimulationManager::clearSelection() {
    for (int nodeId : selectedNodes) {
        Node *node = findNodeById(nodeId);
        if (node) {
            node->setSelected(false);
        }
    }
    selectedNodes.clear();
}

void SimulationManager::selectNode(int nodeId, bool multiSelect) {
    Node *node = findNodeById(nodeId);
    if (!node)
        return;

    if (!multiSelect) {
        clearSelection();
    }

    auto it = std::find(selectedNodes.begin(), selectedNodes.end(), nodeId);

    if (it != selectedNodes.end()) {
        node->setSelected(false);
        selectedNodes.erase(it);
    } else {
        node->setSelected(true);
        selectedNodes.push_back(nodeId);
    }
}

void SimulationManager::handleInput(const Camera2D &camera) {
    Vector2 mousePos = GetMousePosition();
    Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);
    bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    bool leftDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    bool leftReleased = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);
    bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

    if (!shiftDown) {
        if (leftDown) {
            for (auto &ring : rings) {
                ring->updateDraggedNodePositions(worldPos);
            }
        }

        if (leftPressed && !isPainting) {
            bool picked = false;
            for (auto &ring : rings) {
                if (picked)
                    break;

                Node *hitNode = ring->getNodeAt(worldPos, 30.0f);
                if (hitNode) {
                    hitNode->setDragging(true);
                    picked = true;
                }
            }
        }

        if (leftReleased) {
            for (auto &ring : rings) {
                ring->releaseAllDraggedNodes();
            }
        }
    } else {
        for (auto &ring : rings) {
            ring->releaseAllDraggedNodes();
        }
    }

    if (isPainting && leftPressed && !rings.empty()) {
        auto &ring = *rings.front();
        ring.addNode(std::format("P_Node_{}", ring.getNodeCount()));
        Node &n = ring[ring.getNodeCount() - 1];
        n.setPosition(worldPos);
        return;
    }

    if (shiftDown) {
        if (leftPressed) {
            isBoxSelecting = true;
            boxStart = worldPos;
            boxEnd = worldPos;
        } else if (leftDown && isBoxSelecting) {
            boxEnd = worldPos;
        } else if (leftReleased && isBoxSelecting) {
            isBoxSelecting = false;
            if (!IsKeyDown(KEY_LEFT_CONTROL))
                clearSelection();

            Rectangle rect = getBoxSelectionRect();
            for (auto &ring : rings) {
                for (const auto &node : ring->getNodes()) {
                    Vector2 p = node->getPosition();
                    if (p.x >= rect.x && p.x <= rect.x + rect.width && p.y >= rect.y &&
                        p.y <= rect.y + rect.height) {
                        selectNode(node->getId(), true);
                    }
                }
            }
        }
    } else {
        isBoxSelecting = false;
    }

    if (!isPainting && !isBoxSelecting && leftPressed) {
        Node *clickedNode = nullptr;

        for (auto &ring : rings) {
            clickedNode = ring->getNodeAt(worldPos, 30.0f);
            if (clickedNode)
                break;
        }

        if (clickedNode) {
            selectNode(clickedNode->getId(), shiftDown);
            APP_LOG_INFO("Selected Node: {}", clickedNode->getName());
        } else if (!shiftDown) {
            clearSelection();
        }
    }
}

Node *SimulationManager::findNodeById(int nodeId) const {
    auto it = globalNodeMap.find(nodeId);
    if (it != globalNodeMap.end())
        return it->second;
    return nullptr;
}

void SimulationManager::onNodeRemoved(int nodeId) {
    auto it = std::remove(selectedNodes.begin(), selectedNodes.end(), nodeId);
    selectedNodes.erase(it, selectedNodes.end());

    globalNodeMap.erase(nodeId);
}

void SimulationManager::startBenchmark() {
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
        auto &r = *rings.front();
        while (r.getNodeCount() > 0)
            r.removeLastNode();
    }

    SetTargetFPS(0);
    APP_LOG_INFO("GRADUAL BENCHMARK STARTED");
}

void SimulationManager::stopBenchmark() {
    isBenchmarking = false;
    SetTargetFPS(60);

    std::ofstream f("benchmark_results.csv");
    f << "Nodes,AvgFPS,MinFPS,MaxFPS,Physics(ms),Render(ms)\n";
    for (const auto &d : benchResults) {
        f << std::format("{},{:.1f},{:.1f},{:.1f},{:.3f},{:.3f}\n", d.nodeCount, d.fps, d.minFps,
                         d.maxFps, d.physicsMs, d.renderMs);
    }
    f.close();

    APP_LOG_INFO("BENCHMARK COMPLETE. Results saved to benchmark_results.csv");
}

std::string SimulationManager::getBenchmarkStatus() const {
    if (!isBenchmarking)
        return "Idle";
    int nodes = 0;
    if (!rings.empty())
        nodes = rings.front()->getNodeCount();
    return std::format("Stage {}/20 | Nodes: {}", benchStage, nodes);
}

void SimulationManager::updateBenchmarkLogic(float dt) {
    benchTimer += dt;

    if (benchTimer < 3.0f)
        return;

    if (benchTimer < 5.0f) {
        float currentFPS = (float)GetFPS();
        benchAccFPS += currentFPS;
        if (currentFPS < benchMinFPS)
            benchMinFPS = currentFPS;
        if (currentFPS > benchMaxFPS)
            benchMaxFPS = currentFPS;

        benchAccPhys += Profiler::instance().getAverageTime("Sim_Update");
        benchAccRender += Profiler::instance().getAverageTime("Vis_DrawRing");
        benchSamples++;
        return;
    }

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
            data.fps = 0;
            data.minFps = 0;
            data.maxFps = 0;
            data.physicsMs = 0;
            data.renderMs = 0;
        }

        benchResults.push_back(data);

        APP_LOG_INFO("Bench Stage {}: {} nodes, {:.1f} avg FPS (Min: {:.1f})", benchStage,
                     data.nodeCount, data.fps, data.minFps);

        if (benchStage >= 20 || data.minFps < 10) {
            stopBenchmark();
            return;
        }
    }

    benchStage++;
    benchTimer = 0.0f;
    benchSamples = 0;
    benchAccFPS = 0;
    benchMinFPS = 9999.0;
    benchMaxFPS = 0.0;
    benchAccPhys = 0;
    benchAccRender = 0;

    if (rings.empty())
        addRing({(float)GetScreenWidth() / 2.0f, (float)GetScreenHeight() / 2.0f}, 200);
    auto &r = *rings.front();

    constexpr float MARGIN = 100.0f;
    float maxX = (float)GetScreenWidth() - MARGIN;
    float maxY = (float)GetScreenHeight() - MARGIN;

    for (int i = 0; i < 100; ++i) {
        r.addNode(std::format("Bench_{}_{}", benchStage, i));
        Node &n = r[r.getNodeCount() - 1];
        n.setPosition({(float)GetRandomValue((int)MARGIN, (int)maxX),
                       (float)GetRandomValue((int)MARGIN, (int)maxY)});
    }
}