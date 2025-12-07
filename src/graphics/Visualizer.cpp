#include "graphics/Visualizer.h"
#include <cmath>

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
    for (size_t i = 1; i < tokenTrail.size(); ++i) {
        float alpha = static_cast<float>(i) / tokenTrail.size();
        Color col = RED;
        col.a = static_cast<unsigned char>(alpha * 100);
        float thickness = 2.0f + (alpha * 4.0f);
        DrawLineEx(tokenTrail[i - 1], tokenTrail[i], thickness, col);
    }
}

void Visualizer::drawRingGlow(const Ring &ring) {
    const auto &nodes = ring.getNodes();

    for (size_t i = 0; i < nodes.size(); ++i) {
        size_t nextIdx = (i + 1) % nodes.size();
        Vector2 from = nodes[i]->getPosition();
        Vector2 to = nodes[nextIdx]->getPosition();

        // Animated flowing glow effect
        float flowOffset = fmod(animationTime * 50.0f, 20.0f);
        
        // Multiple glow layers for depth
        Color glowColor1 = ColorFromHSV(200.0f, 0.7f, 0.8f);
        glowColor1.a = 20;
        DrawLineEx(from, to, 12.0f, glowColor1);
        
        Color glowColor2 = ColorFromHSV(200.0f, 0.8f, 0.9f);
        glowColor2.a = 40;
        DrawLineEx(from, to, 8.0f, glowColor2);
        
        Color glowColor3 = ColorFromHSV(200.0f, 0.9f, 1.0f);
        glowColor3.a = 60;
        DrawLineEx(from, to, 4.0f, glowColor3);
        
        // Draw flowing particles along the line
        Vector2 dir = {to.x - from.x, to.y - from.y};
        float length = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (length > 0) {
            dir.x /= length;
            dir.y /= length;
            
            for (float d = flowOffset; d < length; d += 20.0f) {
                Vector2 particlePos = {from.x + dir.x * d, from.y + dir.y * d};
                Color particleColor = ColorFromHSV(200.0f, 0.8f, 1.0f);
                particleColor.a = 150;
                DrawCircleV(particlePos, 3.0f, particleColor);
            }
        }
    }
}

void Visualizer::drawRing(const Ring &ring, float dt) {
    animationTime += dt;

    drawRingGlow(ring);
    drawConnections(ring);
    drawDataTransfers(dt);

    const auto &nodes = ring.getNodes();
    const auto *token = ring.getToken();

    for (const auto &node : nodes) {
        drawNode(*node, node->hasTokenPresent());
    }

    drawDataDistribution(ring, {0, 0});

    if (token && !nodes.empty()) {
        size_t fromIdx = 0;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i]->getId() == token->getCurrentNodeId()) {
                fromIdx = i;
                break;
            }
        }

        size_t toIdx = (fromIdx + 1) % nodes.size();
        Vector2 from = nodes[fromIdx]->getPosition();
        Vector2 to = nodes[toIdx]->getPosition();

        float progress = token->getTravelProgress();
        Vector2 tokenPos = {from.x + (to.x - from.x) * progress,
                            from.y + (to.y - from.y) * progress};

        addTokenTrailPoint(tokenPos);
        drawToken(from, to, progress);

        // Spawn particles occasionally
        // if (GetRandomValue(0, 100) < 30) {
        //     spawnParticles(tokenPos, ORANGE, 1);
        // }
    }

}

void Visualizer::drawNode(const Node &node, bool hasToken) {
    Vector2 pos = node.getPosition();

    if (node.getSelected()) {
        drawSelectedNodeHighlight(node);
    }

    float nodeRadius = 30.0f;
    
    // Pulsing effect for nodes with data
    float pulse = 1.0f;
    if (node.getDataCount() > 0) {
        pulse = 1.0f + 0.05f * sin(animationTime * 3.0f); // Reduced pulse
    }
    float actualRadius = nodeRadius * pulse;

    // Dynamic coloring based on token range
    float hue = (float)node.getTokenRangeStart();
    Color nodeColor = ColorFromHSV(hue, 0.8f, 0.9f);
    Color innerColor = ColorFromHSV(hue, 0.5f, 1.0f);
    
    // Outer glow layers for depth
    Color glowOuter = nodeColor;
    glowOuter.a = 40;
    DrawCircleV(pos, actualRadius + 8.0f, glowOuter);
    
    Color glowMid = nodeColor;
    glowMid.a = 80;
    DrawCircleV(pos, actualRadius + 4.0f, glowMid);

    // Main node body with gradient effect
    DrawCircleGradient(pos.x, pos.y, actualRadius, innerColor, nodeColor);
    
    // Rim light effect
    Color rimColor = ColorFromHSV(hue, 0.6f, 1.0f);
    rimColor.a = 180;
    DrawCircleLines(pos.x, pos.y, actualRadius, rimColor);
    DrawCircleLines(pos.x, pos.y, actualRadius - 1, BLACK);

    // Node text with better shadow
    const char *text = node.getName().data();
    int textWidth = MeasureText(text, 16);
    // Shadow
    DrawText(text, pos.x - textWidth / 2 + 2, pos.y - 6, 16, Fade(BLACK, 0.5f));
    // Main text
    DrawText(text, pos.x - textWidth / 2, pos.y - 8, 16, WHITE);
}

void Visualizer::drawToken(Vector2 from, Vector2 to, float progress) {
    Vector2 pos = {from.x + (to.x - from.x) * progress, from.y + (to.y - from.y) * progress};

    // Pulsing token
    float pulse = 1.0f; // No pulsing
    float tokenRadius = 15.0f * pulse;

    // Glow
    Color glowColor = ORANGE;
    glowColor.a = 80;
    DrawCircleV(pos, tokenRadius + 8.0f, glowColor);
    glowColor.a = 150;
    DrawCircleV(pos, tokenRadius + 4.0f, glowColor);

    // Token body
    DrawCircleV(pos, tokenRadius, RED);
    DrawCircleV(pos, tokenRadius * 0.5f, YELLOW);
    DrawCircleLines(pos.x, pos.y, tokenRadius, MAROON);
}

void Visualizer::drawConnections(const Ring &ring) {
    const auto &nodes = ring.getNodes();

    for (size_t i = 0; i < nodes.size(); ++i) {
        size_t nextIdx = (i + 1) % nodes.size();
        Vector2 from = nodes[i]->getPosition();
        Vector2 to = nodes[nextIdx]->getPosition();

        DrawLineEx(from, to, 3.0f, GRAY);
    }
}

void Visualizer::drawDataDistribution(const Ring &ring, Vector2 position) {
    const auto &nodes = ring.getNodes();

    for (size_t i = 0; i < nodes.size(); ++i) {
        Vector2 nodePos = nodes[i]->getPosition();
        int dataCount = nodes[i]->getDataCount();

        if (dataCount > 0) {
            // Draw data count badge
            Color badgeColor = GOLD;
            badgeColor.a = 200;
            DrawCircleV(Vector2{nodePos.x + 25, nodePos.y - 25}, 12.0f, badgeColor);
            DrawCircleLines(nodePos.x + 25, nodePos.y - 25, 12.0f, ORANGE);

            std::string countText = std::to_string(dataCount);
            int textWidth = MeasureText(countText.c_str(), 14);
            DrawText(countText.c_str(), nodePos.x + 25 - textWidth / 2, nodePos.y - 30, 14, BLACK);
        }

        // Draw token range
        int rangeStart = nodes[i]->getTokenRangeStart();
        int rangeEnd = nodes[i]->getTokenRangeEnd();
        std::string rangeText = std::format("[{}°-{}°)", rangeStart, rangeEnd);
        int textWidth = MeasureText(rangeText.c_str(), 12);
        DrawText(rangeText.c_str(), nodePos.x - textWidth / 2, nodePos.y + 40, 12, DARKGRAY);
    }
}

void Visualizer::startDataTransfer(Vector2 from, Vector2 to, std::string key, bool isReplication) {
    DataTransfer transfer;
    transfer.fromPos = from;
    transfer.toPos = to;
    transfer.progress = 0.0f;
    transfer.dataKey = std::move(key);
    transfer.isReplication = isReplication;
    transfer.color = isReplication ? Color{0, 200, 0, 255} : Color{ // Green for replicas
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        255
    };
    activeTransfers.push_back(transfer);
}

void Visualizer::drawDataTransfers(float dt) {
    for (auto it = activeTransfers.begin(); it != activeTransfers.end(); ) {
        it->progress += dt * 0.8f;  // Speed of transfer

        if (it->progress >= 1.0f) {
            it = activeTransfers.erase(it);
            continue;
        }

        // Draw data packet moving with smooth easing
        float easedProgress = it->progress * it->progress * (3.0f - 2.0f * it->progress); // Smoothstep
        Vector2 pos = {
            it->fromPos.x + (it->toPos.x - it->fromPos.x) * easedProgress,
            it->fromPos.y + (it->toPos.y - it->fromPos.y) * easedProgress
        };

        // Trail effect
        for (int i = 1; i <= 3; i++) {
            float trailProgress = easedProgress - (i * 0.05f);
            if (trailProgress > 0) {
                Vector2 trailPos = {
                    it->fromPos.x + (it->toPos.x - it->fromPos.x) * trailProgress,
                    it->fromPos.y + (it->toPos.y - it->fromPos.y) * trailProgress
                };
                Color trailColor = it->color;
                trailColor.a = 40 / i;
                DrawCircleV(trailPos, 8.0f / i, trailColor);
            }
        }

        // Outer glow layers
        Color glowOuter = it->color;
        glowOuter.a = 40;
        DrawCircleV(pos, 16.0f, glowOuter);
        
        Color glowMid = it->color;
        glowMid.a = 80;
        DrawCircleV(pos, 12.0f, glowMid);

        // Main data packet with gradient
        Color innerColor = it->color;
        innerColor.r = std::min(255, innerColor.r + 50);
        innerColor.g = std::min(255, innerColor.g + 50);
        innerColor.b = std::min(255, innerColor.b + 50);
        DrawCircleGradient(pos.x, pos.y, 8.0f, innerColor, it->color);
        DrawCircleLines(pos.x, pos.y, 8.0f, Fade(BLACK, 0.8f));

        // Key label with better visibility
        const char* text = it->dataKey.c_str();
        int textWidth = MeasureText(text, 10);
        // Background for text
        DrawRectangle(pos.x - textWidth/2 - 2, pos.y - 22, textWidth + 4, 12, Fade(BLACK, 0.7f));
        // Text
        DrawText(text, pos.x - textWidth/2, pos.y - 20, 10, WHITE);

        ++it;
    }
}

int Visualizer::checkNodeClick(const Ring &ring) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 mouseScreenPos = GetMousePosition();
        Vector2 mouseWorldPos = GetScreenToWorld2D(mouseScreenPos, camera);

        for (const auto &node : ring.getNodes()) {
            // Assuming a node radius of 30.0f for clickable area
            float nodeRadius = 30.0f;
            if (CheckCollisionPointCircle(mouseWorldPos, node->getPosition(), nodeRadius)) {
                return node->getId();
            }
        }
        // If no node was clicked, return -1
        return -1;
    }
    // If mouse button was not pressed, return -1 to indicate no click event
    return -1;
}

void Visualizer::drawSelectedNodeHighlight(const Node &node) {
    Vector2 pos = node.getPosition();
    
    // Simple, static highlight
    float highlightRadius = 35.0f; // Fixed radius
    
    // Draw a static orange circle behind the node
    DrawCircleV(pos, highlightRadius, Fade(ORANGE, 0.5f)); // Semi-transparent orange fill
    DrawCircleLines(pos.x, pos.y, highlightRadius, ORANGE); // Solid orange outline
    DrawCircleLines(pos.x, pos.y, highlightRadius + 1.0f, ORANGE); // Thicker outline
}