#pragma once
#include "Ring.h"

class Visualizer {
public:
    auto drawRing(const Ring& ring) -> void;
    
private:
    auto drawNode(const Node& node, bool hasToken) -> void;
    auto drawToken(Vector2 from, Vector2 to, float progress) -> void;
    auto drawConnections(const Ring& ring) -> void;
};
