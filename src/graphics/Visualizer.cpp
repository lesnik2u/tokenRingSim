#include "graphics/Visualizer.h"
#include <cmath>

Visualizer::Visualizer() {
    camera.target = {640, 360};
    camera.offset = {640, 360};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
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

void Visualizer::spawnParticles(Vector2 position, Color color, int count) {
    for (int i = 0; i < count; ++i) {
        float angle = (GetRandomValue(0, 360) * PI) / 180.0f;
        float speed = GetRandomValue(20, 60);

        Particle p;
        p.position = position;
        p.velocity = {cos(angle) * speed, sin(angle) * speed};
        p.color = color;
        p.lifetime = 0.0f;
        p.maxLifetime = GetRandomValue(30, 80) / 100.0f;
        p.size = GetRandomValue(2, 5);

        particles.push_back(p);
    }
}

void Visualizer::updateParticles(float dt) {
    for (auto it = particles.begin(); it != particles.end();) {
        it->lifetime += dt;
        it->position.x += it->velocity.x * dt;
        it->position.y += it->velocity.y * dt;
        it->velocity.x *= 0.95f;
        it->velocity.y *= 0.95f;

        if (it->lifetime >= it->maxLifetime) {
            it = particles.erase(it);
        } else {
            ++it;
        }
    }
}

void Visualizer::drawParticles(float dt) {
    updateParticles(dt);

    for (const auto &p : particles) {
        float alpha = 1.0f - (p.lifetime / p.maxLifetime);
        Color col = p.color;
        col.a = static_cast<unsigned char>(alpha * 255);
        DrawCircleV(p.position, p.size, col);
    }
}

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

        // Glowing connection lines
        Color glowColor = SKYBLUE;
        glowColor.a = 30;
        DrawLineEx(from, to, 8.0f, glowColor);
        glowColor.a = 60;
        DrawLineEx(from, to, 4.0f, glowColor);
    }
}

void Visualizer::drawRing(const Ring &ring, float dt) {
    pulseTimer += dt;

    drawRingGlow(ring);
    drawConnections(ring);
    // drawTokenTrail();
    // drawDataTransfers(dt);

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

        // addTokenTrailPoint(tokenPos);
        // drawToken(from, to, progress);

        // Spawn particles occasionally
        if (GetRandomValue(0, 100) < 30) {
            spawnParticles(tokenPos, ORANGE, 1);
        }
    }

    drawParticles(dt);
}

void Visualizer::drawNode(const Node &node, bool hasToken) {
    Vector2 pos = node.getPosition();

    if (node.getSelected()) {
        drawSelectedNodeHighlight(node);
    }

    // Pulsing animation when has token
    float pulseScale = 1.0f;
    if (hasToken) {
        pulseScale = 1.0f + sin(pulseTimer * 8.0f) * 0.15f;
    }

    float nodeRadius = 30.0f * pulseScale;

    // Glow effect
    if (hasToken) {
        Color glowColor = GREEN;
        glowColor.a = 50;
        DrawCircleV(pos, nodeRadius + 10.0f, glowColor);
        glowColor.a = 100;
        DrawCircleV(pos, nodeRadius + 5.0f, glowColor);
    }

    // Gradient effect (fake with circles)
    Color nodeColor = hasToken ? GREEN : SKYBLUE;
    Color innerColor = hasToken ? LIME : WHITE;

    DrawCircleV(pos, nodeRadius, nodeColor);
    DrawCircleV(pos, nodeRadius * 0.6f, innerColor);
    DrawCircleLines(pos.x, pos.y, nodeRadius, BLACK);

    // Node text with shadow
    const char *text = node.getName().data();
    int textWidth = MeasureText(text, 16);
    DrawText(text, pos.x - textWidth / 2 + 1, pos.y - 7, 16, BLACK);
    DrawText(text, pos.x - textWidth / 2, pos.y - 8, 16, DARKBLUE);
}

void Visualizer::drawToken(Vector2 from, Vector2 to, float progress) {
    Vector2 pos = {from.x + (to.x - from.x) * progress, from.y + (to.y - from.y) * progress};

    // Pulsing token
    float pulse = 1.0f + sin(pulseTimer * 10.0f) * 0.2f;
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

void Visualizer::startDataTransfer(Vector2 from, Vector2 to, std::string key) {
    DataTransfer transfer;
    transfer.fromPos = from;
    transfer.toPos = to;
    transfer.progress = 0.0f;
    transfer.dataKey = std::move(key);
    transfer.color = Color{
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        static_cast<unsigned char>(GetRandomValue(100, 255)),
        255
    };
    activeTransfers.push_back(transfer);
}

void Visualizer::drawDataTransfers(float dt) {
    for (auto it = activeTransfers.begin(); it != activeTransfers.end();) {
        it->progress += dt * 0.8f;  // Speed of transfer

        if (it->progress >= 1.0f) {
            it = activeTransfers.erase(it);
            continue;
        }

        // Draw data packet moving
        Vector2 pos = {
            it->fromPos.x + (it->toPos.x - it->fromPos.x) * it->progress,
            it->fromPos.y + (it->toPos.y - it->fromPos.y) * it->progress
        };

        // Glow effect
        Color glowColor = it->color;
        glowColor.a = 80;
        DrawCircleV(pos, 12.0f, glowColor);

        // Data packet
        DrawCircleV(pos, 8.0f, it->color);
        DrawCircleLines(pos.x, pos.y, 8.0f, BLACK);

        // Key label
        const char* text = it->dataKey.c_str();
        int textWidth = MeasureText(text, 10);
        DrawText(text, pos.x - textWidth/2, pos.y - 20, 10, WHITE);
        DrawText(text, pos.x - textWidth/2 + 1, pos.y - 19, 10, BLACK);

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
        selectedNodeId = -1; // No node clicked
    }
    return selectedNodeId;
}

void Visualizer::drawSelectedNodeHighlight(const Node &node) {
    Vector2 pos = node.getPosition();
    float highlightRadius = 35.0f + (sin(GetTime() * 5.0f) * 2.0f); // Pulsating effect
    DrawCircleLines(pos.x, pos.y, highlightRadius, YELLOW);
    DrawCircleLines(pos.x, pos.y, highlightRadius + 2, YELLOW);
}


