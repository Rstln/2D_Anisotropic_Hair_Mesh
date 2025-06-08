#include "Collider.h"

Collider::Collider(glm::vec3 c, float r) : center(c), radius(r) {
    setupMesh();
}

Collider::~Collider() {
    glDeleteVertexArrays(1, &m_VAO);
    glDeleteBuffers(1, &m_VBO);
}

void Collider::draw(const glm::mat4& projection) {
    m_shader->use();
    glLineWidth(5.0f);

    auto model = glm::mat4(1.0f);
    model = glm::translate(model, center);
    model = glm::scale(model, glm::vec3(radius, radius, 1.0f));
    m_shader->setMat4("u_model", model);
    m_shader->setMat4("u_projection", projection);

    glBindVertexArray(m_VAO);
    glDrawArrays(GL_LINE_LOOP, 0, m_VertexCount);
    glBindVertexArray(0);
}

void Collider::setPosition(glm::vec3 newPosition) {
    center = newPosition;
}

glm::vec3 Collider::getPosition() const {
    return center;
}

void Collider::setupMesh() {
    std::vector<float> vertices;
    int numberOfSegments = 50;

    // 只生成圆周上的点
    for (int i = 0; i <= numberOfSegments; ++i) {
        float angle = i * 2.0f * M_PI / numberOfSegments;
        vertices.push_back(cos(angle)); // x
        vertices.push_back(sin(angle)); // y
        vertices.push_back(0.0f);       // z
    }
    m_VertexCount = vertices.size() / 3;

    // VAO 和 VBO 的创建代码保持不变
    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);

    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}