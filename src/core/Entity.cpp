#include "Entity.h"
#include <iostream>

std::ostream &operator<<(std::ostream &os, const Entity &e) {
    os << e.toString();
    return os;
}