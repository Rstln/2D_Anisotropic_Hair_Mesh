#include "GeometryHelpers.h"
#include <algorithm>
#include <cmath>

namespace GeometryHelpers {
    void pushVec3(std::vector<float>& target, const glm::vec3& v) {
        target.push_back(v.x);
        target.push_back(v.y);
        target.push_back(v.z);
    }


    glm::vec3 getPerpendicularVector(const glm::vec3& p1, const glm::vec3& p2, float thickness) {
        glm::vec2 p1_2d = glm::vec2(p1.x, p1.y);
        glm::vec2 p2_2d = glm::vec2(p2.x, p2.y);

        // 计算二维方向向量
        glm::vec2 delta = p2_2d - p1_2d;
        float len2 = glm::dot(delta, delta);
        if (len2 < 1e-12f) {
            return glm::vec3(0.0f);
        }
        glm::vec2 direction_2d = delta / std::sqrt(len2);

        // 计算二维垂直向量：(-y, x) 是一个常见的选择
        glm::vec2 perp_2d = glm::vec2(-direction_2d.y, direction_2d.x);

        // 将二维垂直向量扩展回三维（Z分量为0）并乘以厚度
        return glm::vec3(perp_2d.x, perp_2d.y, 0.0f) * thickness * 0.5f;
    }


    void addQuad(std::vector<float>& target, const glm::vec3& p1, const glm::vec3& p2, float thickness) {
        glm::vec3 perp = getPerpendicularVector(p1, p2, thickness);

        // Triangle 1: p1_top, p1_bottom, p2_top
        pushVec3(target, p1 + perp);
        pushVec3(target, p1 - perp);
        pushVec3(target, p2 + perp);

        // Triangle 2: p2_top, p1_bottom, p2_bottom
        pushVec3(target, p2 + perp);
        pushVec3(target, p1 - perp);
        pushVec3(target, p2 - perp);
    }

    void addColoredQuad(std::vector<float>& target, const glm::vec3& p1, const glm::vec3& p2, float thickness, glm::vec3 clr) {
        glm::vec3 perp = getPerpendicularVector(p1, p2, thickness);

        // Triangle 1: p1_top, p1_bottom, p2_top
        pushVec3(target, p1 + perp);
        pushVec3(target, clr);

        pushVec3(target, p1 - perp);
        pushVec3(target, clr);

        pushVec3(target, p2 + perp);
        pushVec3(target, clr);


        // Triangle 2: p2_top, p1_bottom, p2_bottom
        pushVec3(target, p2 + perp);
        pushVec3(target, clr);

        pushVec3(target, p1 - perp);
        pushVec3(target, clr);

        pushVec3(target, p2 - perp);
        pushVec3(target, clr);
    }


    glm::vec3 get_log_jet_color(float value) {
        float symmetric_max = 1e-3;

        // 关键：缩小 symmetric_max，相当于需要更大的形变才会触发颜色变化
        float adjusted_max = symmetric_max * 2.0f;  // 乘以2，需要2倍的形变才达到同样的颜色

        float normalized_value = std::clamp((value - (-adjusted_max)) / (adjusted_max - (-adjusted_max)), 0.0f, 1.0f);

        float x = normalized_value * 4.0f;

        float r = 0.0f, g = 0.0f, b = 0.0f;

        if (x < 1.0f) { // Blue to Cyan segment
            r = 0.0f;
            g = x;
            b = 1.0f;
        } else if (x < 2.0f) { // Cyan to Green segment
            r = 0.0f;
            g = 1.0f;
            b = 2.0f - x;
        } else if (x < 3.0f) { // Green to Yellow segment
            r = x - 2.0f;
            g = 1.0f;
            b = 0.0f;
        } else { // Yellow to Red segment
            r = 1.0f;
            g = 4.0f - x;
            b = 0.0f;
        }

        return glm::vec3(r, g, b);
    }




} // namespace GeometryHelpers
