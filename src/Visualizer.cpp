#include "Visualizer.h"
#include <cmath>

auto Visualizer::drawRing(const Ring& ring) -> void {
    drawConnections(ring);
    
    const auto& nodes = ring.getNodes();
    const auto* token = ring.getToken();
    
    for (const auto& node : nodes) {
        drawNode(*node, node->hasTokenPresent());
    }
    
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
        
        drawToken(from, to, token->getTravelProgress());
    }
}

auto Visualizer::drawNode(const Node& node, bool hasToken) -> void {
    Vector2 pos = node.getPosition();
    Color nodeColor = hasToken ? GREEN : BLUE;
    
    DrawCircleV(pos, 30.0f, nodeColor);
    DrawCircleLines(pos.x, pos.y, 30.0f, BLACK);
    
    const char* text = node.getName().data();
    int textWidth = MeasureText(text, 16);
    DrawText(text, pos.x - textWidth/2, pos.y - 8, 16, WHITE);
}

auto Visualizer::drawToken(Vector2 from, Vector2 to, float progress) -> void {
    Vector2 pos = {
        from.x + (to.x - from.x) * progress,
        from.y + (to.y - from.y) * progress
    };
    
    DrawCircleV(pos, 15.0f, RED);
    DrawCircleLines(pos.x, pos.y, 15.0f, DARKGRAY);
}

auto Visualizer::drawConnections(const Ring& ring) -> void {
    const auto& nodes = ring.getNodes();
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        size_t nextIdx = (i + 1) % nodes.size();
        Vector2 from = nodes[i]->getPosition();
        Vector2 to = nodes[nextIdx]->getPosition();
        
        DrawLineEx(from, to, 2.0f, LIGHTGRAY);
    }
}
