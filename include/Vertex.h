#ifndef HAIR2D_VERTEX_H
#define HAIR2D_VERTEX_H


#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>


class Vertex {
public:
    glm::vec3 position{glm::vec3(0.0f, 0.0f,0.0f)};
    glm::vec3 prevPosition{glm::vec3(0.0f, 0.0f,0.0f)};
    glm::vec3 velocity{glm::vec3(0.0f, 0.0f,0.0f)};
    glm::vec3 normal{glm::vec3(0.0f, 0.0f,0.0f)};
    float invMass{1.0f};

    explicit Vertex(const glm::vec3& pos, float mass = 1.0f);

    Vertex(Vertex&&) noexcept = default;

    Vertex& operator=(const Vertex& other);
};


#endif
