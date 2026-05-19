#include "XPBDSimulator.h"
#include <algorithm>

XPBDSimulator::XPBDSimulator(Bundle *b_ptr, std::vector<Collider*>& colliders_ptr) : m_bundle(b_ptr) {
    m_colliders.clear();
    m_colliders = colliders_ptr;
    if (!m_bundle) return;

    // 长度约束 lambda：(num_layers - 1) × num_strands
    lambda_length.resize(m_bundle->num_layers - 1);
    for (int i = 0; i < m_bundle->num_layers - 1; i++) {
        lambda_length[i].resize(m_bundle->num_strands, 0.0f);
    }

    // 角度约束 lambda：每个边界顶点（除了端点）都有一个
    lambda_angle_left.resize(m_bundle->num_layers, 0.0f);
    lambda_angle_right.resize(m_bundle->num_layers, 0.0f);
}

void XPBDSimulator::addVel() {

}

void XPBDSimulator::addVel2() {

}

// =========================================================================
// [核心] 物理模拟主循环
// =========================================================================
void XPBDSimulator::substep() {
    if (!m_bundle) return;
    resetLambda();

    // [Step 1] PBD 1 预测位置 (针对当前所有点)
    for (auto& v : m_bundle->left_vertices) {
        if (v->invMass > 0.0f) {
            v->prevPosition = v->position;
            v->velocity += m_gravity * dt;
            v->position += v->velocity * dt;
        }
    }
    for (auto& v : m_bundle->right_vertices) {
        if (v->invMass > 0.0f) {
            v->prevPosition = v->position;
            v->velocity += m_gravity * dt;
            v->position += v->velocity * dt;
        }
    }


    // [Step 2] 自适应逻辑，改变拓扑
    // 如果发生了裂缝增长/生成，内部会创建新顶点
    bool topologyChanged = solveAdaptive();

    if (topologyChanged) {
        m_bundle->refreshVtxMatrix();
    }


    // [Step 3] PBD 2 约束求解
    for (int k = 0; k < iterTimes; ++k) {
        solveHorizontalCollision(); // 这里的碰撞会把裂缝边缘真正推开
        solveAngleConstraint();
        solveLengthConstraint();
    }


    // [Step 4] PBD 3 更新速度
    for (auto& v : m_bundle->left_vertices) {
        if (v->invMass > 0.0f) {
            v->velocity = (v->position - v->prevPosition) / dt * damping;
        }
    }
    for (auto& v : m_bundle->right_vertices) {
        if (v->invMass > 0.0f) {
            v->velocity = (v->position - v->prevPosition) / dt * damping;
        }
    }
}


bool XPBDSimulator::solveAdaptive() {
    if (!m_bundle || m_colliders.empty()) return false;

    bool topologyChanged = false;

    // =========================================================
    // 1. 产生新裂缝：检测 Level 0 底部边
    // =========================================================
    if (m_bundle->cracks.size() == 0) {
        for (auto& collider : m_colliders) {
            // vtx_matrix[0] 结构: [左, 1L, 1R, 2L, 2R, ..., 右]
            for (int j = 0; j < (int)m_bundle->vtx_matrix[0].size() - 1; j += 2) {
                auto vL = m_bundle->vtx_matrix[0][j];
                auto vR = m_bundle->vtx_matrix[0][j + 1];

                float t_local = 0.0f;
                if (checkSegmentCollision(vL, vR, collider, t_local)) {
                    float raw_u = vL->u_rest + t_local * (vR->u_rest - vL->u_rest);

                    // --- 离散化处理 (Snapping) ---
                    // 假设 U_STEP = 0.01f，raw_u = 0.342f -> snapped_u = 0.34f
                    float snapped_u = std::round(raw_u / U_STEP) * U_STEP;

                    // 边界安全检查
                    snapped_u = glm::clamp(snapped_u, U_STEP, 1.0f - U_STEP);

                    std::shared_ptr<Vertex> vL1_bound, vR1_bound;
                    // 使用我们之前写的辅助函数在第 1 层定位 snapped_u 所在的区间
                    findBoundsInLayer(1, snapped_u, vL1_bound, vR1_bound);

                    // --- 唯一性检查 ---
                    if (m_bundle->cracks.find(snapped_u) == m_bundle->cracks.end()) {
                        m_bundle->cracks.emplace(std::piecewise_construct,
                            std::forward_as_tuple(snapped_u),
                            std::forward_as_tuple(snapped_u, m_bundle->vpool, vL, vR, vL1_bound, vR1_bound, m_bundle->num_layers));

                        topologyChanged = true;
                    }
                }
            }
        }
    }

    // =========================================================
    // 2. 更新现有裂缝：延长或合并
    // =========================================================
    auto it = m_bundle->cracks.begin();
    while (it != m_bundle->cracks.end()) {
        float u_val = it->first;
        Crack& crack = it->second;
        bool crackModified = false;

        // 获取当前裂缝尖端所在的层级
        int currentTopLevel = crack.cn.back()->level;

        // --- A. 向上延长 ---
        if (currentTopLevel < m_bundle->num_layers - 1) {
            int nextLevel = currentTopLevel + 1;

            // 关键点：你需要两层的边界信息
            // 1. 当前尖端层的边界（用于分裂同步）
            std::shared_ptr<Vertex> vL_curr, vR_curr;
            findBoundsInLayer(currentTopLevel, u_val, vL_curr, vR_curr);

            // 2. 下一层（目标延伸层）的边界（用于创建新尖端）
            std::shared_ptr<Vertex> vL_next, vR_next;
            findBoundsInLayer(nextLevel, u_val, vL_next, vR_next);

            bool shouldExtend = false;
            // 检查下一层是否发生碰撞
            for (auto& collider : m_colliders) {
                float t_dummy;
                if (checkSegmentCollision(vL_next, vR_next, collider, t_dummy)) {
                    shouldExtend = true;
                    break;
                }
            }

            // 几何角度检查
            auto tmp_angle = calculateOpeningAngle(crack);
            if (!shouldExtend && calculateOpeningAngle(crack) > ANGLE_THRESHOLD) {
                shouldExtend = true;
            }

            if (shouldExtend) {
                float r_ref = m_colliders.empty() ? 0.1f : m_colliders[0]->radius;

                // 修改 Crack::goUpOnce 的签名，传入两层边界
                // 确保旧层分裂时对齐当前层，新层创建时对齐下一层
                if (crack.goUpOnce(vL_curr, vR_curr, vL_next, vR_next)) {
                    topologyChanged = true;
                    crackModified = true;
                }
            }
        }

        // --- B. 向下合并 ---
        if (!crackModified && crack.cn.size() >= 1) {
            // 这里我们复用 shouldMerge 逻辑
            if (shouldMerge(crack)) {
                crack.goDownOnce();
                topologyChanged = true;
                if (crack.cn.empty()) {
                    it = m_bundle->cracks.erase(it);
                    continue;
                }
            }
        }
        ++it;
    }

    return topologyChanged;
}


void XPBDSimulator::solveLengthConstraint() {
    float inv_dt2 = 1.0f / (dt * dt);
    float compliance = alpha_length * inv_dt2;

    const float scale = 0.1f;
    // 假设你在 Bundle 或 Simulator 中维护了一个 lambda 数组
    // m_bundle->lambdasL[layer][strand]

    for (int i = m_bundle->num_layers - 1; i >= 1; i--) {
        for (int j = 0; j < m_bundle->num_strands; j++) {
            float coeff = (float)j / (float)(m_bundle->num_strands - 1);
            float rest_length = m_bundle->L0[i - 1][j];

            // 1. 获取控制点和位置
            ControlPair up = getControlPointsAt(i, coeff);
            ControlPair down = getControlPointsAt(i - 1, coeff);
            float uUp = up.getLocalCoeff(coeff);
            float uDown = down.getLocalCoeff(coeff);

            glm::vec3 qUp = (1.0f - uUp) * up.left->position + uUp * up.right->position;
            glm::vec3 qDown = (1.0f - uDown) * down.left->position + uDown * down.right->position;

            glm::vec3 dir = qUp - qDown;
            float current_len = glm::length(dir);
            if (current_len < 1e-7f) continue;

            float C = current_len - rest_length;
            glm::vec3 n = dir / current_len;

            // 2. XPBD 核心：使用累加的 lambda
            // 注意：每个子步开始前 lambda 应清零，这里取当前子步累积的值
            float& lambda = lambda_length[i-1][j];

            float wDL = down.left->invMass;
            float wDR = down.right->invMass;
            float gradW_DL = -(1.0f - uDown);
            float gradW_DR = -uDown;

            float sum_w_grad_sq = wDL * (gradW_DL * gradW_DL) + wDR * (gradW_DR * gradW_DR);
            if (sum_w_grad_sq < 1e-9f) continue;

            // --- 修正点 2：标准的 XPBD d_lambda 公式 ---
            float d_lambda = (-C - compliance * lambda) / (sum_w_grad_sq + compliance);
            lambda += d_lambda;

            // 3. 计算下层点的位移
            glm::vec3 dpDL = wDL * (gradW_DL * n) * d_lambda;
            glm::vec3 dpDR = wDR * (gradW_DR * n) * d_lambda;

            // --- 修正点 1：更严谨的位移转嫁逻辑 ---
            auto distributeDown = [&](std::shared_ptr<Vertex> p, glm::vec3 dp, int layer_idx) {
                if (p->u <= 0.0f || p->u >= 1.0f) {
                    // 边界点：直接应用位置修改
                    p->position += dp * scale;
                } else {
                    // 裂缝点（内部点）：
                    // 我们不修改它的 position (因为它由同步决定)，而是修改它的 u。
                    // 原理：将世界空间的位移 dp 投影到该层的水平轴 (R-L) 上
                    auto L = m_bundle->vtx_matrix[layer_idx].front();
                    auto R = m_bundle->vtx_matrix[layer_idx].back();
                    glm::vec3 edge = R->position - L->position;
                    float eLen2 = glm::dot(edge, edge);

                    if (eLen2 > 1e-8f) {
                        // 投影：dp 在 edge 方向上的比例变化即为 u 的变化
                        float du = glm::dot(dp, edge) / eLen2;
                        p->u += du;
                        // 保护：不要越过该层的 L/R 边界
                        p->u = glm::clamp(p->u, 0.001f, 0.999f);
                    }

                    // 纵向补偿：由于我们改的是 u (横向)，
                    // 纵向的 dp.y 无法通过 u 吸收，通常由边界 L, R 分担
                    L->position.y += (1.0f - p->u) * dp.y * scale;
                    R->position.y += p->u * dp.y * scale;
                }
            };

            distributeDown(down.left,  dpDL, i - 1);
            distributeDown(down.right, dpDR, i - 1);
        }
    }

    // 4. 同步
    for(auto& [u, crack] : m_bundle->cracks) {
        for(int k = 0; k < (int)crack.cn.size(); k++) {
            crack.syncPositions(k, m_bundle->vtx_matrix[k].front(), m_bundle->vtx_matrix[k].back());
        }
    }
}


void XPBDSimulator::solveAngleConstraint() {
    if (!m_bundle) return;

    float inv_dt2 = 1.0f / (dt * dt);
    float compliance = alpha_angle * inv_dt2;

    // lambda_ptr 用于根据处理的是左边还是右边指向对应的数组
    auto solveSide = [&](std::vector<std::shared_ptr<Vertex>>& vertices, std::vector<float>& lambdas) {
        if (vertices.size() < 3) return;

        // 从上往下遍历，让刚性从根部向梢部传导
        for (int i = (int)vertices.size() - 2; i >= 1; i--) {
            auto& p2 = vertices[i + 1]; // 上
            auto& p1 = vertices[i];     // 中 (角顶点)
            auto& p0 = vertices[i - 1]; // 下

            glm::vec3 e1 = p2->position - p1->position;
            glm::vec3 e2 = p0->position - p1->position;

            // 1. 约束函数 C：2D 叉积
            float C = e1.x * e2.y - e1.y * e2.x;

            // 2. 计算梯度
            glm::vec3 g0 = glm::vec3(-e1.y, e1.x, 0.0f);
            glm::vec3 g2 = glm::vec3(e2.y, -e2.x, 0.0f);
            glm::vec3 g1 = -g0 - g2;

            // 3. 获取权重
            float w0 = p0->invMass;
            float w1 = p1->invMass;
            float w2 = p2->invMass;

            float sum_w_grad_sq = w0 * glm::dot(g0, g0) +
                                  w1 * glm::dot(g1, g1) +
                                  w2 * glm::dot(g2, g2);

            if (sum_w_grad_sq < 1e-9f) continue;

            // --- XPBD 修正：获取并更新当前约束的累积 lambda ---
            // i 是角顶点的层级索引，对应 lambdas 中的位置
            float& lambda = lambdas[i];

            // 4. 计算 XPBD 乘子公式 (包含 alpha * lambda 项)
            float d_lambda = (-C - compliance * lambda) / (sum_w_grad_sq + compliance);
            lambda += d_lambda;

            // 5. 应用修正
            // 注意：标准的 XPBD 不再需要 angle_stiffness，
            // 约束的强弱完全由 alpha_angle (compliance) 决定。
            p0->position += w0 * g0 * d_lambda;
            p1->position += w1 * g1 * d_lambda;
            p2->position += w2 * g2 * d_lambda;
        }
    };

    solveSide(m_bundle->left_vertices, lambda_angle_left);
    solveSide(m_bundle->right_vertices, lambda_angle_right);
}


void XPBDSimulator::solveHorizontalCollision() {
    if (!m_bundle) return;

    for (auto &collider: m_colliders) {
        // A. 处理左边缘
        for (int i = 0; i < m_bundle->left_vertices.size() - 1; i++) {
            solveEdgeColliderCollision(m_bundle->left_vertices[i],
                                       m_bundle->left_vertices[i + 1],
                                       collider);
        }

        // B. 处理右边缘
        for (int i = 0; i < m_bundle->right_vertices.size() - 1; i++) {
            solveEdgeColliderCollision(m_bundle->right_vertices[i],
                                       m_bundle->right_vertices[i + 1],
                                       collider);
        }

        // C. 处理竖直裂缝边 (假设你以后会将裂缝存入 cracks 集合)
        // 裂缝由撕裂时生成的动态顶点对组成
        for (auto &[fst, snd]: m_bundle->cracks) {
            solveCrackCollision(snd, collider);
        }
    }
}


void XPBDSimulator::resetLambda() {
    for (auto& layer_lambdas : lambda_length) {
        std::fill(layer_lambdas.begin(), layer_lambdas.end(), 0.0f);
    }

    std::fill(lambda_angle_left.begin(), lambda_angle_left.end(), 0.0f);
    std::fill(lambda_angle_right.begin(), lambda_angle_right.end(), 0.0f);
}


float XPBDSimulator::pointToLineSegmentDistance(const glm::vec3 &point, const glm::vec3 &lineStart, const glm::vec3 &lineEnd) {
    glm::vec3 v = lineEnd - lineStart;
    glm::vec3 w = point - lineStart;

    float c1 = glm::dot(w, v);
    if (c1 <= 0) {
        // 最近点是lineStart
        return glm::distance(point, lineStart);
    }

    float c2 = glm::dot(v, v);
    if (c2 <= c1) {
        // 最近点是lineEnd
        return glm::distance(point, lineEnd);
    }

    // 最近点在线段内部
    float b = c1 / c2;
    glm::vec3 closestPoint = lineStart + b * v;
    return glm::distance(point, closestPoint);
}


void XPBDSimulator::solveEdgeColliderCollision(std::shared_ptr<Vertex> v0, std::shared_ptr<Vertex> v1, Collider* collider) {
    glm::vec3& x0 = v0->position;
    glm::vec3& x1 = v1->position;

    glm::vec3 colliderPos = collider->center;
    float colliderRadius = collider->radius;

    // 1. 计算投影参数 t
    glm::vec3 ab = x1 - x0;
    float segLenSq = glm::dot(ab, ab);
    if (segLenSq < 1e-8f) return; // 防止退化边

    float t = glm::dot(colliderPos - x0, ab) / segLenSq;
    t = glm::clamp(t, 0.0f, 1.0f);

    // 2. 计算最近点与距离
    glm::vec3 closestPoint = x0 + t * ab;
    glm::vec3 relativeVec = closestPoint - colliderPos;
    float dist = glm::length(relativeVec);

    if (dist < colliderRadius) {
        // 3. 计算碰撞法线与穿透深度
        // 默认法线方向处理：如果圆心在线段上，根据习惯向外推
        glm::vec3 normal = (dist > 1e-6f) ? (relativeVec / dist) : glm::normalize(x0 - colliderPos);
        // 注意：如果圆心完全重叠，这里逻辑可以根据边缘类型微调，或者默认取 vec3(1,0,0)

        float penetrationDepth = colliderRadius - dist;

        // 4. XPBD 分配权重计算 (基于插值权重 t)
        float w0 = v0->invMass;
        float w1 = v1->invMass;
        float weight0 = (1.0f - t);
        float weight1 = t;

        float denominator = w0 * weight0 * weight0 + w1 * weight1 * weight1;

        if (denominator > 0.0f) {
            float s = .1f * penetrationDepth / denominator;
            glm::vec3 dp0 = s * w0 * weight0 * normal;
            glm::vec3 dp1 = s * w1 * weight1 * normal;

            x0 += dp0;
            x1 += dp1;

            // 关键：吸收碰撞法线方向的动能
            // 将 prevPosition 向前推，减少下一帧的隐式速度
            v0->prevPosition += dp0 * 0.5f;
            v1->prevPosition += dp1 * 0.5f;
        }
    }
}




void XPBDSimulator::solveCrackCollision(Crack& crack, Collider* collider) {
    if (crack.cn.size() < 2) return;

    for (int i = 0; i < (int)crack.cn.size() - 1; i++) {
        auto& node0 = crack.cn[i];
        auto& node1 = crack.cn[i+1];

        auto solveSide = [&](std::shared_ptr<Vertex> v0, std::shared_ptr<Vertex> v1, bool isLeft) {
            glm::vec3& x0 = v0->position;
            glm::vec3& x1 = v1->position;

            glm::vec3 ab = x1 - x0;
            float eLen2 = glm::dot(ab, ab);
            if (eLen2 < 1e-8f) return;

            // 1. 计算投影和距离
            float t = glm::dot(collider->center - x0, ab) / eLen2;
            t = glm::clamp(t, 0.0f, 1.0f);
            glm::vec3 closest = x0 + t * ab;
            glm::vec3 rel = closest - collider->center;
            float dist = glm::length(rel);

            if (dist < collider->radius) {
                crack.collision_layer = i;
                float penetration = collider->radius - dist;

                // 法线方向
                glm::vec3 normal;
                if (dist > 1e-6f) {
                    normal = rel / dist;
                } else {
                    normal = glm::vec3(isLeft ? -1.0f : 1.0f, 0.0f, 0.0f);
                }

                // ⭐ 2. 计算需要的横向位移量（在NDC空间）
                // 我们希望裂缝点在横向上移开 penetration 的距离
                // 获取当前层的左右边界
                auto L = m_bundle->vtx_matrix[crack.cn[i]->level].front();
                auto R = m_bundle->vtx_matrix[crack.cn[i]->level].back();
                glm::vec3 edge = R->position - L->position;
                float edgeLen = glm::length(edge);

                if (edgeLen < 1e-6f) return;  // 边退化

                // 横向位移对应的 u 变化量
                // penetration 是世界空间距离，需要投影到横向
                float horizontal_displacement = normal.x * penetration;

                // 转换为 u 的变化：du = displacement / edgeLength
                float du = horizontal_displacement / edgeLen;

                // ⭐ 3. 应用柔和的 u 修正（加入刚度系数）
                const float crack_collision_stiffness = 0.1f;  // 可调节
                du *= crack_collision_stiffness;

                v0->u += du * (1.0f - t);
                v1->u += du * t;

                // 4. 拓扑保护
                float ur = crack.u_rest;
                const float epsilon = 1e-6f;
                if (isLeft) {
                    v0->u = glm::clamp(v0->u, 0.001f, ur - epsilon);
                    v1->u = glm::clamp(v1->u, 0.001f, ur - epsilon);
                } else {
                    v0->u = glm::clamp(v0->u, ur + epsilon, 0.999f);
                    v1->u = glm::clamp(v1->u, ur + epsilon, 0.999f);
                }
            }
        };

        solveSide(node0->left_end, node1->left_end, true);
        solveSide(node0->right_end, node1->right_end, false);
    }

    // 统一同步位置（基于修改后的 u）
    for (int i = 0; i < (int)crack.cn.size(); i++) {
        crack.syncPositions(i,
            m_bundle->vtx_matrix[crack.cn[i]->level].front(),
            m_bundle->vtx_matrix[crack.cn[i]->level].back());
    }
}


ControlPair XPBDSimulator::getControlPointsAt(int level, float coeff) {
    // 1. 收集该层级所有的“断点”
    // 每个断点包含：u坐标，以及在该坐标处提供的顶点指针
    struct BreakPoint {
        float u;
        std::shared_ptr<Vertex> v_as_left;  // 作为左侧区间右边界的点
        std::shared_ptr<Vertex> v_as_right; // 作为右侧区间左边界的点
    };

    std::vector<BreakPoint> points;

    // 放入起始边界
    points.push_back({0.0f, nullptr, m_bundle->left_vertices[level]});
    points.push_back({1.0f, m_bundle->right_vertices[level], nullptr});

    // 遍历所有裂缝，看该层是否有对应的 CrackNode
    for (auto& [u_coord, crack] : m_bundle->cracks) {
        // 查找该裂缝是否涵盖了这一层
        for (auto& node : crack.cn) {
            if (node->level == level) {
                // 在 u_coord 位置，裂缝提供了两个点
                // left_end 是它左边区间的“右边界”
                // right_end 是它右边区间的“左边界”
                points.push_back({u_coord, node->left_end, node->right_end});
                break;
            }
        }
    }

    // 2. 按 u 坐标排序
    std::sort(points.begin(), points.end(), [](const BreakPoint& a, const BreakPoint& b) {
        return a.u < b.u;
    });

    // 3. 寻找 coeff 落在哪个区间 [points[k], points[k+1]]
    for (int k = 0; k < (int)points.size() - 1; k++) {
        if (coeff >= points[k].u && coeff <= points[k+1].u) {
            ControlPair pair;
            pair.left = points[k].v_as_right;   // 当前区间左边界
            pair.right = points[k+1].v_as_left; // 当前区间右边界
            pair.u_left = points[k].u;
            pair.u_right = points[k+1].u;
            return pair;
        }
    }

    // 备选兜底
    return {m_bundle->left_vertices[level], m_bundle->right_vertices[level], 0.0f, 1.0f};
}


bool XPBDSimulator::checkSegmentCollision(std::shared_ptr<Vertex> vL, std::shared_ptr<Vertex> vR, Collider* col, float& t) {
    glm::vec3 pL = vL->position;
    glm::vec3 pR = vR->position;
    glm::vec3 edge = pR - pL;
    float lenSq = glm::dot(edge, edge);

    if (lenSq < 1e-9f) return false;

    // 计算圆心在边上的投影比例 t
    t = glm::dot(col->center - pL, edge) / lenSq;
    t = glm::clamp(t, 0.0f, 1.0f);

    glm::vec3 closest = pL + t * edge;
    return glm::distance(col->center, closest) < col->radius;
}


void XPBDSimulator::findBoundsInLayer(int level, float u, std::shared_ptr<Vertex>& outL, std::shared_ptr<Vertex>& outR) {
    const auto& layer = m_bundle->vtx_matrix[level];
    const float EPS = 1e-5f;

    for (size_t j = 0; j < layer.size() - 1; ++j) {
        // 使用 EPS 包含边界，确保能够找到匹配的区间
        if (u >= layer[j]->u_rest - EPS && u <= layer[j+1]->u_rest + EPS) {
            outL = layer[j];
            outR = layer[j+1];
            return;
        }
    }
    // 降级处理：防止浮点误差
    outL = layer.front();
    outR = layer.back();
}


float XPBDSimulator::calculateOpeningAngle(Crack& crack) {
    if (crack.cn.size() < 2) return 0.0f;

    auto* tipNode = crack.cn.back().get();
    auto* baseNode = crack.cn[crack.cn.size() - 2].get();

    glm::vec3 tip = tipNode->left_end->position;
    glm::vec3 left = baseNode->left_end->position;
    glm::vec3 right = baseNode->right_end->position;

    glm::vec3 dirL = left - tip;
    glm::vec3 dirR = right - tip;

    float lenL = glm::length(dirL);
    float lenR = glm::length(dirR);

    // 安全检查：如果距离太近，无法定义向量方向
    if (lenL < 1e-7f || lenR < 1e-7f) {
        return 0.0f;
    }

    glm::vec3 vL = dirL / lenL;
    glm::vec3 vR = dirR / lenR;

    float dot = glm::clamp(glm::dot(vL, vR), -1.0f, 1.0f);
    float angle = glm::degrees(std::acos(dot));

    // 如果角度计算结果本身是 NaN（预防万一）
    if (std::isnan(angle)) return 0.0f;

    return angle;
}


bool XPBDSimulator::shouldMerge(Crack& crack) {
    if (crack.cn.size() < 2) return false;

    auto* nodeBelow = crack.cn[crack.cn.size() - 2].get();
    // 1. 如果裂开的两点距离小于 1mm，考虑合并
    float dist = glm::distance(nodeBelow->left_end->position, nodeBelow->right_end->position);
    if (dist > 0.02f) return false;

    // 2. 确保没有碰撞体在附近“撑着”
    for (auto& col : m_colliders) {
        if (glm::distance(col->center, nodeBelow->left_end->position) < col->radius * 1.05f) {
            return false;
        }
    }
    return true;
}