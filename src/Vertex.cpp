#include "Vertex.h"

Vertex::Vertex(const glm::vec3& pos, float mass)
        : position(pos), prevPosition(pos), velocity(0.0f), normal(0.0f, 1.0f, 0.0f) {
    invMass = (mass == 0.0f) ? 0.0f : 1.0f / mass;
}


Vertex &Vertex::operator=(const Vertex &other) {
    if (this != &other) {
        position = other.position;
        prevPosition = other.prevPosition;
        velocity = other.velocity;
        normal = other.normal;
        invMass = other.invMass;
    }
    return *this;
}