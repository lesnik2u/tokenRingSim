#include "Entity.h"
#include <iostream>

auto operator<<(std::ostream& os, const Entity& e) -> std::ostream& {
    os << e.toString();
    return os;
}
