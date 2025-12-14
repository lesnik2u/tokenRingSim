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
    bakeNodeTexture();
}

Visualizer::~Visualizer() {
    UnloadRenderTexture(nodeTexture);
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
    animationTime += dt * animationSpeed;

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
    drawDataTransfers(dt * animationSpeed); 
    drawNodesAdditive(nodes, colorMap);
    
    EndBlendMode();

    // 3. Normal Pass (Text & UI elements)
    if (showText) drawNodesText(nodes);
    drawDataDistribution(ring, {0, 0});
    
    // 4. Debug Pass
    if (showVelocity) drawVelocityVectors(nodes);

    PROFILE_END("Vis_DrawRing");
}

void Visualizer::drawConnections(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) {
    // Fast lookup map for positions
    std::unordered_map<int, Vector2> posMap;
    for (const auto& node : nodes) posMap[node->getId()] = node->getPosition();

    LODLevel lod = getLODLevel();
    // High LOD: Thick lines + Glow (Standard Raylib)
    // Med/Low LOD: Thin lines (Batched RL_LINES)
    
    if (lod == LODLevel::HIGH) {
        for (const auto& node : nodes) {
            Vector2 pos1 = node->getPosition();
            
            // Culling
            Vector2 screenPos = GetWorldToScreen2D(pos1, camera);
            if (screenPos.x < -100 || screenPos.x > GetScreenWidth() + 100 || 
                screenPos.y < -100 || screenPos.y > GetScreenHeight() + 100) continue;

            int id1 = node->getId();
            Color col1 = (colorMap.find(id1) != colorMap.end()) ? colorMap.at(id1) : GRAY;

            for (Node* neighbor : node->getNeighbors()) {
                int id2 = neighbor->getId();
                if (id1 < id2 && posMap.count(id2)) { 
                    Vector2 pos2 = posMap.at(id2);
                    Color lineColor = GRAY;
                    if (node->getClusterId() != -1 && node->getClusterId() == neighbor->getClusterId()) {
                        lineColor = col1;
                    }
                    DrawLineEx(pos1, pos2, 4.0f * globalScale, Fade(lineColor, 0.3f)); // Glow
                    DrawLineEx(pos1, pos2, 1.5f * globalScale, Fade(WHITE, 0.6f));     // Core
                }
            }
        }
    } else {
        // Fast Batch Rendering
        rlBegin(RL_LINES);
        for (const auto& node : nodes) {
            Vector2 pos1 = node->getPosition();
            // Less aggressive culling for lines since they span
            Vector2 screenPos = GetWorldToScreen2D(pos1, camera);
            if (screenPos.x < -300 || screenPos.x > GetScreenWidth() + 300 || 
                screenPos.y < -300 || screenPos.y > GetScreenHeight() + 300) continue;

            int id1 = node->getId();
            Color col1 = (colorMap.find(id1) != colorMap.end()) ? colorMap.at(id1) : GRAY;

            for (Node* neighbor : node->getNeighbors()) {
                int id2 = neighbor->getId();
                if (id1 < id2 && posMap.count(id2)) { 
                    Vector2 pos2 = posMap.at(id2);
                    Color lineColor = GRAY;
                    if (node->getClusterId() != -1 && node->getClusterId() == neighbor->getClusterId()) {
                        lineColor = col1;
                    }
                    
                    // Simple white core
                    rlColor4ub(255, 255, 255, 150);
                    rlVertex2f(pos1.x, pos1.y);
                    rlVertex2f(pos2.x, pos2.y);
                    
                    // Colored tint (optional, adds 2x vertices)
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

void Visualizer::drawNodesAdditive(const std::vector<std::unique_ptr<Node>>& nodes, const std::unordered_map<int, Color>& colorMap) {
    Rectangle source = {0, 0, (float)nodeTexture.texture.width, (float)-nodeTexture.texture.height};
    LODLevel lod = getLODLevel();
    
    // Pass 1: Batched Sprites (All LODs)
    // Raylib batches DrawTexture calls automatically if state doesn't change
    for (const auto& node : nodes) {
        Vector2 pos = node->getPosition();
        // Simple frustum culling
        Vector2 screenPos = GetWorldToScreen2D(pos, camera);
        if (screenPos.x < -50 || screenPos.x > GetScreenWidth() + 50 || 
            screenPos.y < -50 || screenPos.y > GetScreenHeight() + 50) continue;

        int id = node->getId();
        Color baseColor = (colorMap.find(id) != colorMap.end()) ? colorMap.at(id) : GRAY;

        float pulse = 1.0f;
        float rotation = 0.0f;
        
        if (animationsEnabled) {
            pulse = 1.0f + 0.2f * sin(animationTime * 3.0f + id);
            rotation = animationTime * 40.0f + id * 10.0f;
        }

        float scale = (lod == LODLevel::LOW) ? 0.5f : 0.8f * pulse;
        scale *= globalScale; // Apply global scaling
        
        Rectangle dest = {pos.x, pos.y, nodeTexture.texture.width * scale, nodeTexture.texture.height * scale};
        Vector2 origin = {dest.width / 2.0f, dest.height / 2.0f};
        
        DrawTexturePro(nodeTexture.texture, source, dest, origin, rotation, baseColor);
    }

    // Pass 2: Batched Lines (Medium/High LOD)
    if (lod != LODLevel::LOW) {
        rlBegin(RL_LINES);
        for (const auto& node : nodes) {
            Vector2 pos = node->getPosition();
            // Culling reuse? For now, re-check or skip optimization for lines
            Vector2 screenPos = GetWorldToScreen2D(pos, camera);
            if (screenPos.x < -50 || screenPos.x > GetScreenWidth() + 50 || 
                screenPos.y < -50 || screenPos.y > GetScreenHeight() + 50) continue;

            float radius = 20.0f * globalScale;
            float rotation = 0.0f;
            if (animationsEnabled) {
                rotation = (animationTime * 40.0f + node->getId() * 10.0f) * DEG2RAD;
            } else {
                 // Optional: Static offset per node ID if we want random static rotation
                 // rotation = (node->getId() * 10.0f) * DEG2RAD;
                 // For now, keep it aligned (0)
            }
            
            // Draw Hexagon manually for batching
            for (int i = 0; i < 6; i++) {
                float angle1 = rotation + (i * 60) * DEG2RAD;
                float angle2 = rotation + ((i + 1) * 60) * DEG2RAD;
                
                Vector2 p1 = { pos.x + cosf(angle1) * radius, pos.y + sinf(angle1) * radius };
                Vector2 p2 = { pos.x + cosf(angle2) * radius, pos.y + sinf(angle2) * radius };
                
                rlColor4ub(255, 255, 255, 200);
                rlVertex2f(p1.x, p1.y);
                rlVertex2f(p2.x, p2.y);
            }
        }
        rlEnd();
    }
    
    // Pass 3: Satellites (High LOD only)
    if (lod == LODLevel::HIGH) {
        for (const auto& node : nodes) {
            int dataCount = node->getDataCount();
            if (dataCount == 0) continue;
            
            Vector2 pos = node->getPosition();
            Vector2 screenPos = GetWorldToScreen2D(pos, camera);
            if (screenPos.x < -50 || screenPos.x > GetScreenWidth() + 50 || 
                screenPos.y < -50 || screenPos.y > GetScreenHeight() + 50) continue;

            int id = node->getId();
            Color baseColor = (colorMap.find(id) != colorMap.end()) ? colorMap.at(id) : GRAY;
            float radius = 20.0f * globalScale;
            float orbitRadius = radius * 1.8f;

            for (int i = 0; i < dataCount; ++i) {
                float angle = (i * 360.0f / dataCount) * DEG2RAD;
                if (animationsEnabled) {
                    angle += animationTime * 2.0f;
                }
                
                Vector2 satPos = { 
                    pos.x + cos(angle) * orbitRadius, 
                    pos.y + sin(angle) * orbitRadius 
                };
                // Use DrawCircleV which is efficient enough for small batches, or could use POINTS
                DrawCircleV(satPos, 3.0f, WHITE);
                DrawCircleV(satPos, 6.0f, Fade(baseColor, 0.5f));
            }
        }
    }
}

void Visualizer::drawNodesText(const std::vector<std::unique_ptr<Node>>& nodes) {
    if (getLODLevel() != LODLevel::HIGH) return; // Optimization

    for (const auto& node : nodes) {
        Vector2 pos = node->getPosition();
        // Culling
        Vector2 screenPos = GetWorldToScreen2D(pos, camera);
        if (screenPos.x < -50 || screenPos.x > GetScreenWidth() + 50 || 
            screenPos.y < -50 || screenPos.y > GetScreenHeight() + 50) continue;

        float radius = 20.0f * globalScale;
        
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

void Visualizer::drawVelocityVectors(const std::vector<std::unique_ptr<Node>>& nodes) {
    rlBegin(RL_LINES);
    rlColor4ub(0, 255, 0, 150); // Green semi-transparent

    for (const auto& node : nodes) {
        if (!node->getMobile()) continue;
        
        Vector2 pos = node->getPosition();
        // Culling
        Vector2 screenPos = GetWorldToScreen2D(pos, camera);
        if (screenPos.x < -50 || screenPos.x > GetScreenWidth() + 50 || 
            screenPos.y < -50 || screenPos.y > GetScreenHeight() + 50) continue;

        Vector2 vel = node->getVelocity();
        float speed = Vector2Length(vel);
        if (speed < 1.0f) continue;

        // Scale vector for visibility
        Vector2 end = { pos.x + vel.x * 0.5f, pos.y + vel.y * 0.5f };
        
        rlVertex2f(pos.x, pos.y);
        rlVertex2f(end.x, end.y);
    }
    rlEnd();
}

int Visualizer::checkNodeClick(const Ring &ring) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 mouseScreenPos = GetMousePosition();
        Vector2 mouseWorldPos = GetScreenToWorld2D(mouseScreenPos, camera);

        Node* clicked = ring.getNodeAt(mouseWorldPos, 30.0f);
        if (clicked) return clicked->getId();
        
        return -1;
    }
    return -1;
}

void Visualizer::drawSelectedNodeHighlight(const Node &node) {
    Vector2 pos = node.getPosition();
    // Note: This is called inside Normal pass, so blend mode is Normal.
    // Need to switch if we want additive glow here?
    // Or just draw lines.
    DrawCircleLines(pos.x, pos.y, 40.0f * globalScale, WHITE);
    DrawCircleLines(pos.x, pos.y, 45.0f * globalScale, Fade(WHITE, 0.5f));
}
