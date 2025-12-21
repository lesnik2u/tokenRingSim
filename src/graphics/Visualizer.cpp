#include "graphics/Visualizer.h"
#include "utils/Logger.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <string>
#include <unordered_map>

static constexpr std::array<Vector2, 6> UNIT_HEXAGON = {{{1.0f, 0.0f},
                                                         {0.5f, 0.86602540378f},
                                                         {-0.5f, 0.86602540378f},
                                                         {-1.0f, 0.0f},
                                                         {-0.5f, -0.86602540378f},
                                                         {0.5f, -0.86602540378f}}};

Visualizer::Visualizer() {
    camera.target = {640, 360};
    camera.offset = {640, 360};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    bakeNodeTexture();
}

Visualizer::~Visualizer() { UnloadRenderTexture(nodeTexture); }

// Renders node texture to a texture mode for better performance
void Visualizer::bakeNodeTexture() {
    nodeTexture = LoadRenderTexture(128, 128);
    BeginTextureMode(nodeTexture);
    ClearBackground(Color{0, 0, 0, 0});

    DrawCircleGradient(64, 64, 50.0f, Fade(WHITE, 0.6f), Fade(WHITE, 0.0f));
    DrawPoly(Vector2{64, 64}, 6, 20.0f, 0.0f, Fade(WHITE, 0.9f));
    DrawPolyLinesEx(Vector2{64, 64}, 6, 20.0f, 0.0f, 3.0f, WHITE);

    EndTextureMode();
    SetTextureFilter(nodeTexture.texture, TEXTURE_FILTER_BILINEAR);
}

// Adjusts detail level based on zoom
void Visualizer::updateLOD() {
    if (forceHighDetail) {
        cachedLOD = LODLevel::HIGH;
        return;
    }

    float z = camera.zoom;
    float bufferHigh = 0.05f;
    float bufferMed = 0.03f;

    switch (cachedLOD) {
    case LODLevel::HIGH:
        if (z < lodThresholdHigh - bufferHigh)
            cachedLOD = LODLevel::MEDIUM;
        break;
    case LODLevel::MEDIUM:
        if (z > lodThresholdHigh)
            cachedLOD = LODLevel::HIGH;
        else if (z < lodThresholdMedium - bufferMed)
            cachedLOD = LODLevel::LOW;
        break;
    case LODLevel::LOW:
        if (z > lodThresholdMedium)
            cachedLOD = LODLevel::MEDIUM;
        break;
    }
}

// Processes camera movement and zoom
void Visualizer::handleInput() {
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        float zoomIncrement = 0.1f;
        camera.zoom += wheel * zoomIncrement;
        if (camera.zoom < 0.1f)
            camera.zoom = 0.1f;
        if (camera.zoom > 3.0f)
            camera.zoom = 3.0f;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        isPanning = true;
        lastMousePos = GetMousePosition();
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
        isPanning = false;
    }

    if (isPanning) {
        Vector2 currentMousePos = GetMousePosition();
        Vector2 delta = {currentMousePos.x - lastMousePos.x, currentMousePos.y - lastMousePos.y};
        camera.target.x -= delta.x / camera.zoom;
        camera.target.y -= delta.y / camera.zoom;
        lastMousePos = currentMousePos;
    }

    if (IsKeyPressed(KEY_R)) {
        camera.target = {(float)GetScreenWidth() / 2.0f, (float)GetScreenHeight() / 2.0f};
        camera.zoom = 1.0f;
    }
}

void Visualizer::beginCamera() { BeginMode2D(camera); }
void Visualizer::endCamera() { EndMode2D(); }

void Visualizer::drawRingGlow(const Ring &ring) {}

// Main entry point for drawing a ring
void Visualizer::drawRing(const Ring &ring, float dt) {
    PROFILE_START("Vis_DrawRing");
    updateLOD();

    animationTime += dt * animationSpeed;
    if (animationTime > 10000.0f)
        animationTime -= 10000.0f;

    const std::vector<std::unique_ptr<Node>> &nodes = ring.getNodes();
    if (nodes.empty()) {
        PROFILE_END("Vis_DrawRing");
        return;
    }

    if (ring.getTopologyVersion() != lastTopologyVersion ||
        ring.getSelectionVersion() != lastSelectionVersion) {
        cachedColorMap.clear();
        cachedColorMap.reserve(nodes.size());

        for (const std::unique_ptr<Node> &node : nodes) {
            int cid = node->getClusterId();
            Color col = GRAY;
            if (cid != -1)
                col = ColorFromHSV((cid * 67) % 360, 0.8f, 0.9f);
            if (node->getSelected())
                col = WHITE;
            cachedColorMap[node->getId()] = col;
        }
        lastTopologyVersion = ring.getTopologyVersion();
        lastSelectionVersion = ring.getSelectionVersion();
    }
    const std::unordered_map<int, Color> &colorMap = cachedColorMap;

    std::vector<Vector2> screenPositions;
    std::vector<uint8_t> visibilityCache;
    screenPositions.reserve(nodes.size());
    visibilityCache.reserve(nodes.size());

    float sw = (float)GetScreenWidth();
    float sh = (float)GetScreenHeight();
    const float pad = 100.0f;

    for (const std::unique_ptr<Node> &node : nodes) {
        Vector2 sp = GetWorldToScreen2D(node->getPosition(), camera);
        screenPositions.push_back(sp);

        uint8_t visible =
            (sp.x > -pad && sp.x < sw + pad && sp.y > -pad && sp.y < sh + pad) ? 1 : 0;
        visibilityCache.push_back(visible);
    }

    BeginBlendMode(BLEND_ADDITIVE);

    drawConnections(nodes, colorMap, screenPositions, visibilityCache);
    drawDataTransfers(dt * animationSpeed);
    drawNodesAdditive(nodes, colorMap, screenPositions, visibilityCache);

    EndBlendMode();

    if (showText)
        drawNodesText(nodes, screenPositions, visibilityCache);
    drawDataDistribution(ring, {0, 0});

    if (showVelocity)
        drawVelocityVectors(nodes, screenPositions, visibilityCache);

    PROFILE_END("Vis_DrawRing");
}

// Draws lines between connected nodes
void Visualizer::drawConnections(const std::vector<std::unique_ptr<Node>> &nodes,
                                 const std::unordered_map<int, Color> &colorMap,
                                 const std::vector<Vector2> &screenPositions,
                                 const std::vector<uint8_t> &visibilityCache) {
    LODLevel lod = getLODLevel();

    if (lod == LODLevel::HIGH) {
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (!visibilityCache[i])
                continue;

            const std::unique_ptr<Node> &node = nodes[i];
            Vector2 pos1 = node->getPosition();
            int id1 = node->getId();

            auto it1 = colorMap.find(id1);
            Color col1 = (it1 != colorMap.end()) ? it1->second : GRAY;

            for (Node *neighbor : node->getNeighbors()) {
                assert(neighbor && "Neighbor cannot be null");

                int id2 = neighbor->getId();
                if (id1 < id2) {
                    Vector2 pos2 = neighbor->getPosition();
                    Color lineColor = GRAY;
                    if (node->getClusterId() != -1 &&
                        node->getClusterId() == neighbor->getClusterId()) {
                        lineColor = col1;
                    }
                    DrawLineEx(pos1, pos2, 4.0f * globalScale, Fade(lineColor, 0.3f));
                    DrawLineEx(pos1, pos2, 1.5f * globalScale, Fade(WHITE, 0.6f));
                }
            }
        }
    } else {
        rlBegin(RL_LINES);
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (!visibilityCache[i])
                continue;

            const std::unique_ptr<Node> &node = nodes[i];
            Vector2 pos1 = node->getPosition();
            int id1 = node->getId();

            auto it1 = colorMap.find(id1);
            Color col1 = (it1 != colorMap.end()) ? it1->second : GRAY;

            for (Node *neighbor : node->getNeighbors()) {
                assert(neighbor && "Neighbor cannot be null");

                int id2 = neighbor->getId();
                if (id1 < id2) {
                    Vector2 pos2 = neighbor->getPosition();
                    Color lineColor = GRAY;
                    if (node->getClusterId() != -1 &&
                        node->getClusterId() == neighbor->getClusterId()) {
                        lineColor = col1;
                    }

                    rlColor4ub(255, 255, 255, 150);
                    rlVertex2f(pos1.x, pos1.y);
                    rlVertex2f(pos2.x, pos2.y);

                    if (lod == LODLevel::MEDIUM) {
                        rlColor4ub(lineColor.r, lineColor.g, lineColor.b, 100);
                        rlVertex2f(pos1.x, pos1.y);
                        rlVertex2f(pos2.x, pos2.y);
                    }
                }
            }
        }
        rlEnd();
    }
}

// Draws the main visual body of nodes
void Visualizer::drawNodesAdditive(const std::vector<std::unique_ptr<Node>> &nodes,
                                   const std::unordered_map<int, Color> &colorMap,
                                   const std::vector<Vector2> &screenPositions,
                                   const std::vector<uint8_t> &visibilityCache) {
    Rectangle source = {0, 0, (float)nodeTexture.texture.width, (float)-nodeTexture.texture.height};
    LODLevel lod = getLODLevel();

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (!visibilityCache[i])
            continue;

        const std::unique_ptr<Node> &node = nodes[i];
        Vector2 pos = node->getPosition();
        int id = node->getId();

        auto it = colorMap.find(id);
        Color baseColor = (it != colorMap.end()) ? it->second : GRAY;

        float pulse = 1.0f;
        float rotation = 0.0f;

        if (animationsEnabled) {
            pulse = 1.0f + 0.2f * sin(animationTime * 3.0f + id);
            rotation = animationTime * 40.0f + id * 10.0f;
        }

        float scale = (lod == LODLevel::LOW) ? 0.5f : 0.8f * pulse;
        scale *= globalScale;

        Rectangle dest = {pos.x, pos.y, nodeTexture.texture.width * scale,
                          nodeTexture.texture.height * scale};
        Vector2 origin = {dest.width / 2.0f, dest.height / 2.0f};

        DrawTexturePro(nodeTexture.texture, source, dest, origin, rotation, baseColor);
    }

    if (lod != LODLevel::LOW) {
        rlBegin(RL_LINES);
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (!visibilityCache[i])
                continue;

            const std::unique_ptr<Node> &node = nodes[i];
            Vector2 pos = node->getPosition();
            float radius = 20.0f * globalScale;
            float rotation = 0.0f;
            if (animationsEnabled) {
                rotation = (animationTime * 40.0f + node->getId() * 10.0f) * DEG2RAD;
            }

            float cr = cosf(rotation);
            float sr = sinf(rotation);

            for (int k = 0; k < 6; k++) {
                const Vector2 &v1 = UNIT_HEXAGON[k];
                const Vector2 &v2 = UNIT_HEXAGON[(k + 1) % 6];

                Vector2 p1 = {pos.x + radius * (v1.x * cr - v1.y * sr),
                              pos.y + radius * (v1.x * sr + v1.y * cr)};
                Vector2 p2 = {pos.x + radius * (v2.x * cr - v2.y * sr),
                              pos.y + radius * (v2.x * sr + v2.y * cr)};

                rlColor4ub(255, 255, 255, 200);
                rlVertex2f(p1.x, p1.y);
                rlVertex2f(p2.x, p2.y);
            }
        }
        rlEnd();
    }

    if (lod == LODLevel::HIGH) {
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (!visibilityCache[i])
                continue;

            const std::unique_ptr<Node> &node = nodes[i];
            int dataCount = (int)node->getDataCount();
            if (dataCount == 0)
                continue;

            Vector2 pos = node->getPosition();
            int id = node->getId();

            auto it = colorMap.find(id);
            Color baseColor = (it != colorMap.end()) ? it->second : GRAY;

            float radius = 20.0f * globalScale;
            float orbitRadius = radius * 1.8f;

            for (int k = 0; k < dataCount; ++k) {
                float angle = (k * 360.0f / dataCount) * DEG2RAD;
                if (animationsEnabled) {
                    angle += animationTime * 2.0f;
                }

                Vector2 satPos = {pos.x + cosf(angle) * orbitRadius,
                                  pos.y + sinf(angle) * orbitRadius};
                DrawCircleV(satPos, 3.0f * globalScale, WHITE);
                DrawCircleV(satPos, 6.0f * globalScale, Fade(baseColor, 0.5f));
            }
        }
    }
}

// Renders node names
void Visualizer::drawNodesText(const std::vector<std::unique_ptr<Node>> &nodes,
                               const std::vector<Vector2> &screenPositions,
                               const std::vector<uint8_t> &visibilityCache) {
    if (getLODLevel() != LODLevel::HIGH)
        return;

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (!visibilityCache[i])
            continue;

        const std::unique_ptr<Node> &node = nodes[i];
        Vector2 pos = node->getPosition();
        float radius = 20.0f * globalScale;

        const std::string &label = node->getName();
        int textWidth = MeasureText(label.c_str(), 10);
        DrawText(label.c_str(), (int)(pos.x - textWidth / 2), (int)(pos.y + radius + 5), 10, WHITE);

        if (node->getSelected()) {
            drawSelectedNodeHighlight(*node);
        }
    }
}

void Visualizer::drawDataDistribution(const Ring &ring, Vector2 position) {}

// Initializes a moving data point visualization
void Visualizer::startDataTransfer(Vector2 from, Vector2 to, std::string key, bool isReplication) {
    DataTransfer transfer;
    transfer.fromPos = from;
    transfer.toPos = to;
    transfer.progress = 0.0f;
    transfer.dataKey = std::move(key);
    transfer.isReplication = isReplication;
    transfer.color = isReplication ? GREEN : SKYBLUE;
    activeTransfers.push_back(transfer);
}

// Updates and draws active data movements
void Visualizer::drawDataTransfers(float dt) {
    for (auto it = activeTransfers.begin(); it != activeTransfers.end();) {
        it->progress += dt * 1.5f;

        if (it->progress >= 1.0f) {
            it = activeTransfers.erase(it);
            continue;
        }

        float t = it->progress;
        Vector2 pos = {it->fromPos.x + (it->toPos.x - it->fromPos.x) * t,
                       it->fromPos.y + (it->toPos.y - it->fromPos.y) * t};

        DrawCircleV(pos, 6.0f * globalScale, it->color);
        DrawCircleV(pos, 12.0f * globalScale, Fade(it->color, 0.5f));

        Vector2 tail = {it->fromPos.x + (it->toPos.x - it->fromPos.x) * (t - 0.1f),
                        it->fromPos.y + (it->toPos.y - it->fromPos.y) * (t - 0.1f)};
        if (t > 0.1f)
            DrawLineEx(tail, pos, 4.0f * globalScale, Fade(it->color, 0.5f));

        ++it;
    }
}

// Shows movement directions for debugging
void Visualizer::drawVelocityVectors(const std::vector<std::unique_ptr<Node>> &nodes,
                                     const std::vector<Vector2> &screenPositions,
                                     const std::vector<uint8_t> &visibilityCache) {
    rlBegin(RL_LINES);
    rlColor4ub(0, 255, 0, 150);

    for (size_t i = 0; i < nodes.size(); ++i) {
        if (!visibilityCache[i])
            continue;

        const std::unique_ptr<Node> &node = nodes[i];
        if (!node->getMobile())
            continue;

        Vector2 pos = node->getPosition();
        Vector2 vel = node->getVelocity();
        float speed = Vector2Length(vel);
        if (speed < 1.0f)
            continue;

        Vector2 end = {pos.x + vel.x * 0.5f, pos.y + vel.y * 0.5f};

        rlVertex2f(pos.x, pos.y);
        rlVertex2f(end.x, end.y);
    }
    rlEnd();
}

void Visualizer::drawSelectionBox(Rectangle rect) {
    DrawRectangleLinesEx(rect, 2.0f, WHITE);
    DrawRectangleRec(rect, Fade(WHITE, 0.2f));
}

// Highlight circle for selected node
void Visualizer::drawSelectedNodeHighlight(const Node &node) {
    Vector2 pos = node.getPosition();
    DrawCircleLines((int)pos.x, (int)pos.y, 40.0f * globalScale, WHITE);
    DrawCircleLines((int)pos.x, (int)pos.y, 45.0f * globalScale, Fade(WHITE, 0.5f));
}