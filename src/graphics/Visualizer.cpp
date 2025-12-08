#include "graphics/Visualizer.h"
#include "utils/Logger.h"
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
    bakeNodeTexture();
}

void Visualizer::bakeNodeTexture() {
    nodeTexture = LoadRenderTexture(128, 128);
    BeginTextureMode(nodeTexture);
    ClearBackground(Color{0,0,0,0});
    
    DrawCircleGradient(64, 64, 50.0f, Fade(WHITE, 0.6f), Fade(WHITE, 0.0f));
    DrawPoly(Vector2{64, 64}, 6, 20.0f, 0.0f, Fade(WHITE, 0.9f));
    DrawPolyLinesEx(Vector2{64, 64}, 6, 20.0f, 0.0f, 3.0f, WHITE);
    
    EndTextureMode();
    SetTextureFilter(nodeTexture.texture, TEXTURE_FILTER_BILINEAR);
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

void Visualizer::drawTokenTrail() {}
void Visualizer::drawRingGlow(const Ring &ring) {}

void Visualizer::drawRing(const Ring &ring, float dt) {
    PROFILE_START("Vis_DrawRing");
    animationTime += dt;

    const auto &nodes = ring.getNodes();
    if (nodes.empty()) { PROFILE_END("Vis_DrawRing"); return; }

    // 1. Pre-calculate Colors (Map ID -> Color)
    std::unordered_map<int, Color> colorMap;
    for (const auto& node : nodes) {
        int cid = node->getClusterId();
        Color col = GRAY;
        if (cid != -1) col = ColorFromHSV((cid * 67) % 360, 0.8f, 0.9f);
        if (node->getSelected()) col = WHITE;
        colorMap[node->getId()] = col;
    }

    // 2. Additive Pass (Everything Glowing)
    BeginBlendMode(BLEND_ADDITIVE);
    
    drawConnections(nodes, colorMap);
    drawDataTransfers(dt); 
    drawNodesAdditive(nodes, colorMap);
    
    EndBlendMode();

    // 3. Normal Pass (Text & UI elements)
    drawNodesText(nodes);
    drawDataDistribution(ring, {0, 0});

    PROFILE_END("Vis_DrawRing");
}

void Visualizer::drawConnections(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) {
    // Fast lookup map for positions
    std::unordered_map<int, Vector2> posMap;
    for (const auto& node : nodes) posMap[node->getId()] = node->getPosition();

    for (const auto& node : nodes) {
        Vector2 pos1 = node->getPosition();
        int id1 = node->getId();
        // Safe lookup
        Color col1 = (colorMap.find(id1) != colorMap.end()) ? colorMap.at(id1) : GRAY;

        for (Node* neighbor : node->getNeighbors()) {
            int id2 = neighbor->getId();
            if (id1 < id2) { 
                if (posMap.find(id2) != posMap.end()) {
                    Vector2 pos2 = posMap.at(id2);
                    
                    // Determine color
                    Color lineColor = GRAY;
                    if (node->getClusterId() != -1 && node->getClusterId() == neighbor->getClusterId()) {
                        lineColor = col1;
                    }

                    DrawLineEx(pos1, pos2, 4.0f, Fade(lineColor, 0.3f)); // Glow
                    DrawLineEx(pos1, pos2, 1.5f, Fade(WHITE, 0.6f));     // Core
                }
            }
        }
    }
}

void Visualizer::drawNodesAdditive(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) {
    Rectangle source = {0, 0, (float)nodeTexture.texture.width, (float)-nodeTexture.texture.height};
    
    for (const auto& node : nodes) {
        Vector2 pos = node->getPosition();
        float radius = 20.0f;
        int id = node->getId();
        Color baseColor = (colorMap.find(id) != colorMap.end()) ? colorMap.at(id) : GRAY;

        float pulse = 1.0f + 0.2f * sin(animationTime * 3.0f + id);
        float rotation = animationTime * 40.0f + id * 10.0f;
        
        float scale = 0.8f * pulse;
        Rectangle dest = {pos.x, pos.y, nodeTexture.texture.width * scale, nodeTexture.texture.height * scale};
        Vector2 origin = {dest.width / 2.0f, dest.height / 2.0f};
        
        // Draw Core
        DrawTexturePro(nodeTexture.texture, source, dest, origin, rotation, baseColor);
        
        // Draw Border
        DrawPolyLinesEx(pos, 6, radius, rotation, 2.0f, Fade(WHITE, 0.8f));
        
        // Draw Satellites
        int dataCount = node->getDataCount();
        if (dataCount > 0) {
            float orbitRadius = radius * 1.8f;
            for (int i = 0; i < dataCount; ++i) {
                float angle = animationTime * 2.0f + (i * 360.0f / dataCount) * DEG2RAD;
                Vector2 satPos = { 
                    pos.x + cos(angle) * orbitRadius, 
                    pos.y + sin(angle) * orbitRadius 
                };
                DrawCircleV(satPos, 3.0f, WHITE);
                DrawCircleV(satPos, 6.0f, Fade(baseColor, 0.5f));
            }
        }
    }
}

void Visualizer::drawNodesText(const std::vector<std::unique_ptr<Node>>& nodes) {
    for (const auto& node : nodes) {
        Vector2 pos = node->getPosition();
        float radius = 20.0f;
        
        // Draw Text
        const std::string& label = node->getName();
        int textWidth = MeasureText(label.c_str(), 10);
        DrawText(label.c_str(), pos.x - textWidth/2, pos.y + radius + 5, 10, WHITE);
        
        if (node->getSelected()) {
            drawSelectedNodeHighlight(*node);
        }
    }
}

void Visualizer::drawToken(Vector2 from, Vector2 to, float progress) {
}

void Visualizer::drawDataDistribution(const Ring &ring, Vector2 position) {
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
    for (auto it = activeTransfers.begin(); it != activeTransfers.end(); ) {
        it->progress += dt * 1.5f; 

        if (it->progress >= 1.0f) {
            it = activeTransfers.erase(it);
            continue;
        }

        float t = it->progress;
        Vector2 pos = {
            it->fromPos.x + (it->toPos.x - it->fromPos.x) * t,
            it->fromPos.y + (it->toPos.y - it->fromPos.y) * t
        };

        DrawCircleV(pos, 6.0f, it->color);
        DrawCircleV(pos, 12.0f, Fade(it->color, 0.5f));
        
        Vector2 tail = {
            it->fromPos.x + (it->toPos.x - it->fromPos.x) * (t - 0.1f),
            it->fromPos.y + (it->toPos.y - it->fromPos.y) * (t - 0.1f)
        };
        if (t > 0.1f) DrawLineEx(tail, pos, 4.0f, Fade(it->color, 0.5f));

        ++it;
    }
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
    // Note: This is called inside Normal pass, so blend mode is Normal.
    // Need to switch if we want additive glow here?
    // Or just draw lines.
    DrawCircleLines(pos.x, pos.y, 40.0f, WHITE);
    DrawCircleLines(pos.x, pos.y, 45.0f, Fade(WHITE, 0.5f));
}
