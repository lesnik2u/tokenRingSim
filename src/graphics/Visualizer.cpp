#include "graphics/Visualizer.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <string>

Visualizer::Visualizer() {
    camera.target = {640, 360};
    camera.offset = {640, 360};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    animationTime = 0.0f;
}

void Visualizer::handleInput() {
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        float zoomIncrement = 0.1f;
        camera.zoom += wheel * zoomIncrement;
        if (camera.zoom < 0.1f) camera.zoom = 0.1f;
        if (camera.zoom > 3.0f) camera.zoom = 3.0f;
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
        camera.target = {640, 360};
        camera.zoom = 1.0f;
    }
}

void Visualizer::beginCamera() { BeginMode2D(camera); }
void Visualizer::endCamera() { EndMode2D(); }

void Visualizer::addTokenTrailPoint(Vector2 point) {
    tokenTrail.push_back(point);
    if (tokenTrail.size() > 50) {
        tokenTrail.erase(tokenTrail.begin());
    }
}

void Visualizer::drawTokenTrail() {
    // Optional: Cyberpunk trail
}

void Visualizer::drawRingGlow(const Ring &ring) {
    // Disabled
}

void Visualizer::drawRing(const Ring &ring, float dt) {
    animationTime += dt;

    drawConnections(ring);
    drawDataTransfers(dt);

    const auto &nodes = ring.getNodes();
    const auto *token = ring.getToken();

    for (const auto &node : nodes) {
        drawNode(*node, node->hasTokenPresent());
    }

    drawDataDistribution(ring, {0, 0});

    // Draw Token (Energy Spark)
    if (token && !nodes.empty()) {
        size_t fromIdx = 0;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i]->getId() == token->getCurrentNodeId()) {
                fromIdx = i;
                break;
            }
        }

        size_t toIdx = (fromIdx + 1) % nodes.size();
        
        // Find actual next node from logic if possible, but here we just interpolation visual
        // Wait, we can't find 'toIdx' linearly if topology is random.
        // Visualizer doesn't know the token's path.
        // But token logic in Ring.cpp calculated it.
        // We can just snap to node? Or use prev/next from token?
        // For now, if we don't know target, just draw at node.
        // But Ring.cpp updates token position with `updateTravel`.
        // It linearly interpolates between current and next.
        // But Visualizer needs to know WHO is next.
        // Ring.cpp doesn't expose 'nextNodeId' of token publicly.
        // However, `Token` has `travelProgress`.
        // If we assume the token travels along the *bond*, we can look at neighbors.
        // But simpler: just draw it on the node for now, or pulsing.
        // OR: Use the previous code's assumption?
        // Previous code assumed `nodes[i+1]`. This is WRONG for emergent.
        // Fix: Just draw token at `currentNode` position if progress < 0.5, or assume it moves.
        // Actually, `Ring` handles logical movement.
        // Visualizer should just draw it at `node->getPosition()` if `hasToken`.
        // `drawNode` handles `hasToken`.
    }
}

void Visualizer::drawNode(const Node &node, bool hasToken) {
    Vector2 pos = node.getPosition();
    float radius = 20.0f;

    // Determine Color based on Cluster ID
    Color baseColor = GRAY;
    int cid = node.getClusterId();
    if (cid != -1) {
        // Procedural neon colors
        float hue = (cid * 67) % 360;
        baseColor = ColorFromHSV(hue, 0.8f, 0.9f);
    }
    if (node.getSelected()) baseColor = WHITE;

    // --- Cyberpunk Rendering (Additive) ---
    BeginBlendMode(BLEND_ADDITIVE);

    // 1. Energy Glow (Pulsing)
    float pulse = 1.0f + 0.2f * sin(animationTime * 3.0f + node.getId());
    if (hasToken) pulse *= 1.5f; // Token makes it surge
    
    DrawCircleGradient(pos.x, pos.y, radius * 3.0f * pulse, Fade(baseColor, 0.3f), BLANK);
    
    // 2. Core Geometry (Rotating Hexagon)
    float rotation = animationTime * 40.0f + node.getId() * 10.0f;
    if (hasToken) rotation *= 3.0f; // Spin faster with token
    
    // Inner fill
    DrawPoly(pos, 6, radius, rotation, Fade(baseColor, 0.7f));
    // Wireframe rim
    DrawPolyLinesEx(pos, 6, radius, rotation, 2.0f, WHITE);
    
    // 3. Data Satellites
    int dataCount = node.getDataCount();
    if (dataCount > 0) {
        float orbitRadius = radius * 1.8f;
        float orbitSpeed = 2.0f;
        
        for (int i = 0; i < dataCount; ++i) {
            float angle = animationTime * orbitSpeed + (i * 360.0f / dataCount) * DEG2RAD;
            Vector2 satPos = { 
                pos.x + cos(angle) * orbitRadius, 
                pos.y + sin(angle) * orbitRadius 
            };
            
            DrawCircleV(satPos, 3.0f, WHITE);
            DrawCircleV(satPos, 6.0f, Fade(baseColor, 0.5f));
            // Trail?
        }
    }

    EndBlendMode();

    // 4. Text Labels (Normal Blend)
    // Only show text on hover or selection to reduce clutter
    // Or show small ID
    if (node.getSelected() || dataCount > 0) {
        std::string label = std::string(node.getName());
        int textWidth = MeasureText(label.c_str(), 10);
        DrawText(label.c_str(), pos.x - textWidth/2, pos.y + radius + 5, 10, WHITE);
    }
    
    if (node.getSelected()) {
        drawSelectedNodeHighlight(node);
    }
}

void Visualizer::drawToken(Vector2 from, Vector2 to, float progress) {
    // Not used currently, logic inside drawNode
}

void Visualizer::drawConnections(const Ring &ring) {
    const auto &nodes = ring.getNodes();
    if (nodes.empty()) return;

    std::unordered_map<int, Vector2> posMap;
    std::unordered_map<int, Color> colorMap;
    
    for (const auto& node : nodes) {
        posMap[node->getId()] = node->getPosition();
        
        int cid = node->getClusterId();
        Color col = GRAY;
        if (cid != -1) col = ColorFromHSV((cid * 67) % 360, 0.8f, 0.9f);
        colorMap[node->getId()] = col;
    }

    BeginBlendMode(BLEND_ADDITIVE);

    for (const auto& node : nodes) {
        Vector2 pos1 = node->getPosition();
        int id1 = node->getId();
        Color col1 = colorMap[id1];

        for (int id2 : node->getNeighbors()) {
            if (id1 < id2) { // Draw once
                if (posMap.find(id2) != posMap.end()) {
                    Vector2 pos2 = posMap[id2];
                    Color col2 = colorMap[id2];
                    
                    // Interpolate color? Use col1 if same cluster
                    Color lineColor = (node->getClusterId() == -1) ? GRAY : col1;
                    
                    // Beam effect
                    DrawLineEx(pos1, pos2, 4.0f, Fade(lineColor, 0.3f)); // Glow
                    DrawLineEx(pos1, pos2, 1.5f, Fade(WHITE, 0.6f));     // Core
                }
            }
        }
    }
    
    EndBlendMode();
}

void Visualizer::drawDataDistribution(const Ring &ring, Vector2 position) {
    // Removed or simplified
}

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

void Visualizer::drawDataTransfers(float dt) {
    BeginBlendMode(BLEND_ADDITIVE);
    for (auto it = activeTransfers.begin(); it != activeTransfers.end(); ) {
        it->progress += dt * 1.5f; // Faster

        if (it->progress >= 1.0f) {
            it = activeTransfers.erase(it);
            continue;
        }

        float t = it->progress;
        Vector2 pos = {
            it->fromPos.x + (it->toPos.x - it->fromPos.x) * t,
            it->fromPos.y + (it->toPos.y - it->fromPos.y) * t
        };

        // Data Packet (Energy Bolt)
        DrawCircleV(pos, 6.0f, it->color);
        DrawCircleV(pos, 12.0f, Fade(it->color, 0.5f));
        
        // Trail
        Vector2 tail = {
            it->fromPos.x + (it->toPos.x - it->fromPos.x) * (t - 0.1f),
            it->fromPos.y + (it->toPos.y - it->fromPos.y) * (t - 0.1f)
        };
        if (t > 0.1f) {
            DrawLineEx(tail, pos, 4.0f, Fade(it->color, 0.5f));
        }

        ++it;
    }
    EndBlendMode();
}

int Visualizer::checkNodeClick(const Ring &ring) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 mouseScreenPos = GetMousePosition();
        Vector2 mouseWorldPos = GetScreenToWorld2D(mouseScreenPos, camera);

        for (const auto &node : ring.getNodes()) {
            if (CheckCollisionPointCircle(mouseWorldPos, node->getPosition(), 30.0f)) {
                return node->getId();
            }
        }
        return -1;
    }
    return -1;
}

void Visualizer::drawSelectedNodeHighlight(const Node &node) {
    Vector2 pos = node.getPosition();
    BeginBlendMode(BLEND_ADDITIVE);
    DrawCircleLines(pos.x, pos.y, 40.0f, WHITE);
    DrawCircleLines(pos.x, pos.y, 45.0f, Fade(WHITE, 0.5f));
    EndBlendMode();
}
