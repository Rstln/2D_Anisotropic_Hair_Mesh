#ifndef HAIR2D_COLLIDER_H
#define HAIR2D_COLLIDER_H

#include "Shader.h"
#include "GeometryHelpers.h"

class Collider {
public:
    Collider(glm::vec3 c, float r);
    ~Collider();

    void setPosition(glm::vec3 new_pos);
    glm::vec3 getPosition() const;
    void draw(const glm::mat4& projection);


    // for rendering x is not [-1, 1]
    glm::vec3 center;
    // for simulation [-1, 1]
    glm::vec3 pos;


    float radius;
    glm::vec4 lineColor{glm::vec4(0.0f, 0.0f,1.0f,  1.0f)};
    float thickness{0.1f};
    std::vector<float> m_fatCircleVertices;
    unsigned int m_VAO{0}, m_VBO{0};
    Shader* m_shader{nullptr};

    void setupMesh();
};





#endif //HAIR2D_COLLIDER_H
