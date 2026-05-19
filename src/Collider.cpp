#include "Collider.h"

using namespace GeometryHelpers;

Collider::Collider(glm::vec3 c, float r) : center(c), radius(r) {
    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);

    // 绑定VAO并设置顶点属性
    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    // 初始分配空间，但数据会在 setupMesh 中填充
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // 解绑
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    setupMesh();
}

Collider::~Collider() {
    glDeleteVertexArrays(1, &m_VAO);
    glDeleteBuffers(1, &m_VBO);
}

void Collider::draw(const glm::mat4& projection) {
    m_shader->use();
    m_shader->setVec3("lineColor", lineColor);

    auto model = glm::mat4(1.0f);
    model = glm::translate(model, center);
    model = glm::scale(model, glm::vec3(radius, radius, 1.0f));
    m_shader->setMat4("u_model", model);
    m_shader->setMat4("u_projection", projection);

    glBindVertexArray(m_VAO);
    glDrawArrays(GL_TRIANGLES, 0, m_fatCircleVertices.size() / 3);
    glBindVertexArray(0);
}

void Collider::setPosition(glm::vec3 newPosition) {
    center = newPosition;
}

glm::vec3 Collider::getPosition() const {
    return center;
}

void Collider::setupMesh() {
    m_fatCircleVertices.clear(); // 清除旧数据
    int numberOfSegments = 50; // 圆的段数，越多越圆滑

    // 生成圆周上的点对，并为每对点生成一个四边形
    for (int i = 0; i < numberOfSegments; ++i) {
        float angle1 = i * 2.0f * M_PI / numberOfSegments;
        float angle2 = (i + 1) * 2.0f * M_PI / numberOfSegments;

        // 获取当前线段的两个端点（单位圆上的点，Z为0）
        // 这里生成的是半径为1的圆的顶点。实际半径会在draw函数的model矩阵中处理。
        glm::vec3 p1 = glm::vec3(cos(angle1), sin(angle1), 0.0f);
        glm::vec3 p2 = glm::vec3(cos(angle2), sin(angle2), 0.0f);

        // 为这个小线段添加四边形
        addQuad(m_fatCircleVertices, p1, p2, thickness);
    }

    // 更新 VBO 数据
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(GL_ARRAY_BUFFER, m_fatCircleVertices.size() * sizeof(float), m_fatCircleVertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}