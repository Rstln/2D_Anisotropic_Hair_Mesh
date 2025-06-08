#include "XPBDSimulator.h"

XPBDSimulator::XPBDSimulator(Bundle *b_ptr, Collider *c_ptr) : m_bundle(b_ptr), m_collider(c_ptr) {

}



void XPBDSimulator::addVel() {
    m_bundle->layers[1][1].position += glm::vec3(.5f, 0.2f, 0.0f);
    m_bundle->layers[1][0].position += glm::vec3(.5f, 0.2f, 0.0f);
}

void XPBDSimulator::substep() {
    // 1.predict position
    for (int i = 1; i < m_bundle->layers.size(); i++) {
        for (auto& vtx : m_bundle->layers[i]) {
            vtx.prevPosition = vtx.position;
            vtx.velocity += glm::vec3(0.0f, -9.8f, 0.0f) * dt;
            vtx.position += vtx.velocity * dt;
            // now .position is predicted position tilde_x
        }
    }

    for (int i = 1; i < m_bundle->layers.size(); i++) {
        // 2. solve constraint(s), length
        for (int cnt = 0; cnt < iterTimes; cnt++) {
            solveLengthConstraint(i);
            if (i < m_bundle->layers.size() - 1)
                solveAngleConstraint(i);

            // solveCollision(i);
            solveLineCollision(i);
        }
    }

    // 3. update vel
    for (int i = 1; i < m_bundle->layers.size(); i++) {
        for (auto& vtx : m_bundle->layers[i]) {
            vtx.velocity = damping * (vtx.position - vtx.prevPosition) / dt;
        }
    }
}


void XPBDSimulator::solveLengthConstraint(int i) {
    auto& x0 = m_bundle->layers[i-1][0].position;
    auto& x1 = m_bundle->layers[i-1][1].position;
    auto& x2 = m_bundle->layers[i][0].position;
    auto& x3 = m_bundle->layers[i][1].position;

    float coeff = 0.0f;
    // num_strands == num_constraints_length
    for (int j = 0; j < m_bundle->num_strands; j++) {
        coeff += m_bundle->step;
        auto tmp1 = coeff * x1 + (1-coeff) * x0;
        auto tmp2 = coeff * x3 + (1-coeff) * x2;
        auto dv = tmp2 - tmp1;
        float diff = glm::length(dv) - m_bundle->L0[i][j];
        float C = diff * diff;
        auto dir = glm::normalize(dv);

        auto grad_x0 = - dir * (1-coeff) * 2.0f * diff;
        auto grad_x1 = - dir * coeff * 2.0f * diff;
        auto grad_x2 = dir * (1-coeff) * 2.0f * diff;
        auto grad_x3 = dir * coeff * 2.0f * diff;

        float grad_square = m_bundle->layers[i-1][0].invMass * glm::dot(grad_x0, grad_x0) +
                m_bundle->layers[i-1][1].invMass * glm::dot(grad_x1, grad_x1) +
                m_bundle->layers[i][0].invMass  * glm::dot(grad_x2, grad_x2) +
                m_bundle->layers[i][1].invMass  * glm::dot(grad_x3, grad_x3);
        float d_lambda = (- C - alpha * lambda) / (grad_square + alpha);

        auto dx_x0 = m_bundle->layers[i-1][0].invMass * grad_x0 * d_lambda;
        auto dx_x1 = m_bundle->layers[i-1][1].invMass * grad_x1 * d_lambda;
        auto dx_x2 = m_bundle->layers[i][0].invMass * grad_x2 * d_lambda;
        auto dx_x3 = m_bundle->layers[i][1].invMass * grad_x3 * d_lambda;
        lambda += d_lambda;

        if (m_bundle->layers[i-1][0].invMass > 0.0f) {
            x0 += dx_x0;
            x1 += dx_x1;
        }
        else {
//            x2 += dx_x2;
//            x3 += dx_x3;
        }
        x2 += dx_x2;
        x3 += dx_x3;
    }
}


void XPBDSimulator::solveCollision(int i) {
    for (int idx = i; idx <= i+1; idx++) {
        if (idx >= 0 && idx < m_bundle->layers.size()) {
            for (auto& p : m_bundle->layers[idx]) {
                glm::vec2 particlePos(p.position.x, p.position.y);
                glm::vec2 colliderPos = m_collider->getPosition();
                float colliderRadius = m_collider->radius;


                float dist = glm::distance(particlePos, colliderPos);
                if (dist < colliderRadius) {
                    float penetration = colliderRadius - dist;
                    glm::vec2 collisionNormal = (dist > 0.0001f) ? glm::normalize(particlePos - colliderPos) : glm::vec2(0.0f, 1.0f);

                    glm::vec2 correction = penetration * collisionNormal;

                    p.position.x += correction.x;
                    p.position.y += correction.y;


                    glm::vec2 vel_before( (p.position.x - p.prevPosition.x) / dt, (p.position.y - p.prevPosition.y) / dt );
                    float vn_scalar = glm::dot(vel_before, collisionNormal);
                    glm::vec2 vn_vec = vn_scalar * collisionNormal;
                    glm::vec2 vt_vec = vel_before - vn_vec;

                    vt_vec *= glm::max(0.0f, 1.0f - friction_coeff);

                    vn_vec *= -restitution_coeff;

                    glm::vec2 new_vel = vn_vec + vt_vec;

                    p.prevPosition.x = p.position.x - new_vel.x * dt;
                    p.prevPosition.y = p.position.y - new_vel.y * dt;
                }
            }
        }
    }
}


void XPBDSimulator::solveLineCollision(int i) {
    auto& p0 = m_bundle->layers[i-1][0]; // Top-Left control particle
    auto& p1 = m_bundle->layers[i-1][1]; // Top-Right control particle
    auto& p2 = m_bundle->layers[i][0];   // Bottom-Left control particle
    auto& p3 = m_bundle->layers[i][1];   // Bottom-Right control particle

    float coeff = 0.0f;
    for (int j = 0; j < m_bundle->num_strands; j++) {
        coeff += m_bundle->step; // 步进系数，用于在 x0-x1 和 x2-x3 之间插值

        // 1. 计算当前插值发丝的端点位置 (tmp1, tmp2)
        //    这里我们直接用控制点的位置
        auto tmp1 = coeff * p1.position + (1.0f - coeff) * p0.position; // 插值出的上端点
        auto tmp2 = coeff * p3.position + (1.0f - coeff) * p2.position; // 插值出的下端点

        // --- 碰撞处理开始 ---

        // 2. 对发丝边 tmp1-tmp2 进行碰撞检测
        glm::vec2 pos1(tmp1.x, tmp1.y);
        glm::vec2 pos2(tmp2.x, tmp2.y);
        glm::vec2 colliderCenter = m_collider->getPosition();

        glm::vec2 edge = pos2 - pos1;
        // 注意：这里需要检查 edge 的长度是否过小，避免除以零
        float edge_len_sq = glm::dot(edge, edge);
        if (edge_len_sq < 1e-9) continue;

        float t = glm::dot(colliderCenter - pos1, edge) / edge_len_sq;
        if (t < 0.0f || t > 1.0f) continue;


        glm::vec2 closestPointQ = pos1 + t * edge; // 这是在插值发丝上发生碰撞的点

        float dist = glm::distance(closestPointQ, colliderCenter);
        float colliderRadius = m_collider->radius;


        if (dist >= colliderRadius) continue; // 没有碰撞，处理下一根发丝

        // 3. 计算碰撞修正向量
        float penetration = colliderRadius - dist;
        glm::vec2 collisionNormal = (dist > 1e-6f) ? glm::normalize(closestPointQ - colliderCenter) : glm::vec2(0.0f, 1.0f);
        glm::vec2 collisionCorrection = penetration * collisionNormal;

        // 4. 计算反向传播的权重 (核心！)
        //    点 Q 是四个控制点的线性组合:
        //    Q = (1-t)*tmp1 + t*tmp2
        //      = (1-t)*[ (1-coeff)*p0 + coeff*p1 ] + t*[ (1-coeff)*p2 + coeff*p3 ]
        //    所以，Q对每个控制点位置的偏导数（即权重）是:
        float w_p0 = (1.0f - t) * (1.0f - coeff);
        float w_p1 = (1.0f - t) * coeff;
        float w_p2 = t * (1.0f - coeff);
        float w_p3 = t * coeff;

        float invMass0 = p0.invMass;
        float invMass1 = p1.invMass;
        float invMass2 = p2.invMass;
        float invMass3 = p3.invMass;

        // --- 变化点 2: 分母现在包含所有四个顶点的贡献 ---
        float denominator = invMass0 * w_p0 * w_p0 +
                            invMass1 * w_p1 * w_p1 +
                            invMass2 * w_p2 * w_p2 +
                            invMass3 * w_p3 * w_p3;
        if (denominator < 1e-9) continue;

        // --- 变化点 3: 计算并应用对上层顶点的修正 ---
        glm::vec2 p0_correction = collisionCorrection * (w_p0 * invMass0 / denominator);
        p0.position.x += p0_correction.x;
        p0.position.y += p0_correction.y;

        glm::vec2 p1_correction = collisionCorrection * (w_p1 * invMass1 / denominator);
        p1.position.x += p1_correction.x;
        p1.position.y += p1_correction.y;

        // 计算并应用对下层顶点的修正 (逻辑不变, 但分母变了)
        glm::vec2 p2_correction = collisionCorrection * (w_p2 * invMass2 / denominator);
        p2.position.x += p2_correction.x;
        p2.position.y += p2_correction.y;

        glm::vec2 p3_correction = collisionCorrection * (w_p3 * invMass3 / denominator);
        p3.position.x += p3_correction.x;
        p3.position.y += p3_correction.y;

        // 6. 处理摩擦 (同样需要作用于所有四个顶点)
        // --- 变化点 4: 碰撞点速度现在插值所有四个控制点速度 ---
        glm::vec2 vel0 = glm::vec2(p0.position - p0.prevPosition) / dt;
        glm::vec2 vel1 = glm::vec2(p1.position - p1.prevPosition) / dt;
        glm::vec2 vel2 = glm::vec2(p2.position - p2.prevPosition) / dt;
        glm::vec2 vel3 = glm::vec2(p3.position - p3.prevPosition) / dt;
        glm::vec2 vel_q = vel0 * w_p0 + vel1 * w_p1 + vel2 * w_p2 + vel3 * w_p3;

        float vn_scalar = glm::dot(vel_q, collisionNormal);
        glm::vec2 vt_vec = vel_q - vn_scalar;

        float vt_len = glm::length(vt_vec);

        if (vt_len > 1e-6) {
            glm::vec2 friction_dir = -glm::normalize(vt_vec);
            float friction_dist = glm::min(vt_len * dt, penetration * friction_coeff);
            glm::vec2 frictionCorrection = friction_dir * friction_dist;

            // --- 变化点 5: 摩擦修正也分配给所有四个顶点 ---
            glm::vec2 p0_fric_corr = frictionCorrection * (w_p0 * invMass0 / denominator);
            p0.position.x += p0_fric_corr.x;
            p0.position.y += p0_fric_corr.y;

            glm::vec2 p1_fric_corr = frictionCorrection * (w_p1 * invMass1 / denominator);
            p1.position.x += p1_fric_corr.x;
            p1.position.y += p1_fric_corr.y;

            glm::vec2 p2_fric_corr = frictionCorrection * (w_p2 * invMass2 / denominator);
            p2.position.x += p2_fric_corr.x;
            p2.position.y += p2_fric_corr.y;

            glm::vec2 p3_fric_corr = frictionCorrection * (w_p3 * invMass3 / denominator);
            p3.position.x += p3_fric_corr.x;
            p3.position.y += p3_fric_corr.y;
        }

    } // for each hair strand (j)
}


void XPBDSimulator::solveAngleConstraint(int i) {
    auto& x0 = m_bundle->layers[i-1][0].position;
    auto& x1 = m_bundle->layers[i-1][1].position;
    auto& x2 = m_bundle->layers[i][0].position;
    auto& x3 = m_bundle->layers[i][1].position;
    auto& x4 = m_bundle->layers[i+1][0].position;
    auto& x5 = m_bundle->layers[i+1][1].position;

    // ========= left half =============
    auto v1 = glm::normalize(x0-x2);
    auto v2 = glm::normalize(x4-x2);
    float currentAngle0 = std::acos(std::max(-1.0f, std::min(1.0f, glm::dot(v1, v2))));
    float diff0 = currentAngle0 - M_PI;
    if (std::fabs(diff0) > M_PI / 3) {
        float C0 = diff0 * diff0;
        auto perp1 = glm::vec3(-v1.y, v1.x, 0.0f);
        auto perp2 = glm::vec3(v2.y, -v2.x, 0.0f);
        float cross = v1.x * v2.y - v1.y * v2.x;
        float sign = (cross < 0.0f) ? -1.0f : 1.0f;

        auto grad_C0_x0 = 2.0f * diff0 * sign * perp1;
        auto grad_C0_x4 = 2.0f * diff0 * sign * perp2;
        auto grad_C0_x2 = - grad_C0_x0 - grad_C0_x4;

        float grad_C0_square = m_bundle->layers[i-1][0].invMass * glm::dot(grad_C0_x0, grad_C0_x0) +
                               m_bundle->layers[i][0].invMass * glm::dot(grad_C0_x2, grad_C0_x2)
                               + m_bundle->layers[i+1][0].invMass * glm::dot(grad_C0_x4, grad_C0_x4);
        float d_lambda0 = (- C0 - alpha_angle * lambda0) / (grad_C0_square + alpha_angle);
        if (std::abs(C0) < 1e-6)
            d_lambda0 = 0;
        auto dx_x0 = m_bundle->layers[i-1][0].invMass * grad_C0_x0 * d_lambda0;
        auto dx_x2 = m_bundle->layers[i][0].invMass * grad_C0_x2 * d_lambda0;
        auto dx_x4 = m_bundle->layers[i+1][0].invMass * grad_C0_x4 * d_lambda0;
        lambda0 += d_lambda0;
        if (m_bundle->layers[i-1][0].invMass > 0.0f)
            x0 += dx_x0;
        x2 += dx_x2;
        x4 += dx_x4;
    }

    // ========= right half =============
    v1 = x1-x3;
    v2 = x5-x3;
    float currentAngle1 = std::acos(std::max(-1.0f, std::min(1.0f, glm::dot(v1, v2))));
    float diff1 = currentAngle1 - M_PI;
    if (std::fabs(diff1) > M_PI / 3) {
        float C1 = diff1 * diff1;
        auto perp1 = glm::vec3(-v1.y, v1.x, 0.0f);
        auto perp2 = glm::vec3(v2.y, -v2.x, 0.0f);
        auto cross = v1.x * v2.y - v1.y * v2.x;
        auto sign = (cross < 0.0f) ? -1.0f : 1.0f;

        auto grad_C1_x1 = 2.0f * diff0 * sign * perp1;
        auto grad_C1_x5 = 2.0f * diff0 * sign * perp2;
        auto grad_C1_x3 = - grad_C1_x1 - grad_C1_x5;


        float grad_C1_square = m_bundle->layers[i-1][1].invMass * glm::dot(grad_C1_x1, grad_C1_x1) +
                               m_bundle->layers[i][1].invMass * glm::dot(grad_C1_x3, grad_C1_x3)
                               + m_bundle->layers[i+1][1].invMass * glm::dot(grad_C1_x5, grad_C1_x5);
        float d_lambda1 = (- C1 - alpha_angle * lambda1) / (grad_C1_square + alpha_angle);
        if (std::abs(C1) < 1e-6)
            d_lambda1 = 0;
        auto dx_x1 = m_bundle->layers[i-1][1].invMass * grad_C1_x1 * d_lambda1;
        auto dx_x3 = m_bundle->layers[i][1].invMass * grad_C1_x3 * d_lambda1;
        auto dx_x5 = m_bundle->layers[i+1][1].invMass * grad_C1_x5 * d_lambda1;
        lambda1 += d_lambda1;

        if (m_bundle->layers[i-1][0].invMass > 0.0f)
            x1 += dx_x1;
        x3 += dx_x3;
        x5 += dx_x5;
    }
}