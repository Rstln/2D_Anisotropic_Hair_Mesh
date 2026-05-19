#ifndef HAIR2D_XPBDSIMULATOR_H
#define HAIR2D_XPBDSIMULATOR_H

#include "Bundle.h"
#include "Collider.h"



struct ControlPair {
    std::shared_ptr<Vertex> left;
    std::shared_ptr<Vertex> right;
    float u_left;  // 左控制点的原始 u 坐标
    float u_right; // 右控制点的原始 u 坐标

    // 计算发丝相对于这两个控制点的局部插值系数
    float getLocalCoeff(float globalCoeff) const {
        if (std::abs(u_right - u_left) < 1e-6f) return 0.0f;
        return (globalCoeff - u_left) / (u_right - u_left);
    }
};


class XPBDSimulator {
public:
    const float U_STEP = 0.01f;
    const int iterTimes{10};
    const float dt{1.0f / 500.0f};
    const float damping{0.97f};

    // Compliance (顺应性)
    const float alpha_length{1e-11f};         // 长度约束刚度
    const float alpha_angle{1e-2f};    // 弯曲约束刚度

    const float ANGLE_THRESHOLD{50.0f};

    const glm::vec3 m_gravity = glm::vec3(0.0f, -9.8*5.0f, 0.0f);

    Bundle* m_bundle{nullptr};
    std::vector<Collider*> m_colliders;
    // ===================================================

    std::vector<std::vector<float>> lambda_length;  // [layer_pair][strand]
    std::vector<float> lambda_angle_left;            // [vertex]
    std::vector<float> lambda_angle_right;


    // ===================================================

    XPBDSimulator(Bundle* b_ptr, std::vector<Collider*>& colliders_ptr);

    void addVel();
    void addVel2();
    void substep();

    // ================= XPBD Constraints =================
    // 1. 纵向碰撞与拓扑改变
    bool solveAdaptive();

    // 2. 长度约束
    void solveLengthConstraint();

    // 3. 角度约束
    void solveAngleConstraint();

    // 4. 横向碰撞
    void solveHorizontalCollision();


private:
    void resetLambda();
    float pointToLineSegmentDistance(const glm::vec3& point,
                                const glm::vec3& lineStart,
                                const glm::vec3& lineEnd);
    void solveEdgeColliderCollision(std::shared_ptr<Vertex> v0, std::shared_ptr<Vertex> v1, Collider* collider);
    void solveCrackCollision(Crack& crack, Collider* collider);
    ControlPair getControlPointsAt(int level, float coeff);
    bool checkSegmentCollision(std::shared_ptr<Vertex> vL, std::shared_ptr<Vertex> vR, Collider* col, float& t);
    void findBoundsInLayer(int level, float u, std::shared_ptr<Vertex>& outL, std::shared_ptr<Vertex>& outR);
    float calculateOpeningAngle(Crack& crack);
    bool shouldMerge(Crack& crack);
};

#endif
