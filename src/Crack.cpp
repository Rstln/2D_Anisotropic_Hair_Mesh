#include "Crack.h"

// 构造函数：现在一次性初始化 Level 0 (分裂) 和 Level 1 (共享尖端)
Crack::Crack(float u_rest, VPool* pool,
             std::shared_ptr<Vertex> vL_bound0, std::shared_ptr<Vertex> vR_bound0,
             std::shared_ptr<Vertex> vL_bound1, std::shared_ptr<Vertex> vR_bound1,
             int MAX_LAYER)
    : u_rest(u_rest), m_pool(pool), MAX_LAYER(MAX_LAYER) {

    // --- 1. 创建 Level 0：分裂的两个顶点 ---
    glm::vec3 pos0 = (1.0f - u_rest) * vL_bound0->position + u_rest * vR_bound0->position;
    glm::vec3 prevPos0 = (1.0f - u_rest) * vL_bound0->prevPosition + u_rest * vR_bound0->prevPosition;

    auto vLeft0 = m_pool->createVertex(u_rest, pos0);
    auto vRight0 = m_pool->createVertex(u_rest, pos0);
    vLeft0->prevPosition = prevPos0;
    vRight0->prevPosition = prevPos0;
    vLeft0->u = u_rest - epsilon_u;
    vRight0->u = u_rest + epsilon_u;
    // 设置 invMass：让裂缝点显得“轻”，不拽动边界
    vLeft0->invMass = 10.0f;
    vRight0->invMass = 10.0f;

    cn.push_back(std::make_unique<CrackNode>(0, vLeft0, vRight0));

    // 立即同步，将 u 转换为真实的 position
    syncPositions(0, vL_bound0, vR_bound0);

    // --- 2. 创建 Level 1：共享的尖端顶点 ---
    glm::vec3 pos1 = (1.0f - u_rest) * vL_bound1->position + u_rest * vR_bound1->position;
    glm::vec3 prevPos1 = (1.0f - u_rest) * vL_bound1->prevPosition + u_rest * vR_bound1->prevPosition;

    auto vShared1 = m_pool->createVertex(u_rest, pos1);
    vShared1->prevPosition = prevPos1;
    vShared1->invMass = 10.0f;

    cn.push_back(std::make_unique<CrackNode>(1, vShared1, vShared1));
}

// 向上延伸：逻辑不变，将当前的共享尖端分裂，并在更高层创建新尖端
// 传入两层边界：当前尖端层 (level_i) 和 即将生成的更高层 (level_i+1)
bool Crack::goUpOnce(std::shared_ptr<Vertex> vL_i, std::shared_ptr<Vertex> vR_i,
                     std::shared_ptr<Vertex> vL_next, std::shared_ptr<Vertex> vR_next) {
    if (cn.empty()) return false;

    CrackNode* oldTip = cn.back().get();

    // 1. 分裂旧尖端 (处于 level_i)
    auto vRight = m_pool->createVertex(u_rest, oldTip->left_end->position);
    vRight->u += epsilon_u;
    vRight->prevPosition = oldTip->left_end->prevPosition;
    vRight->invMass = 10.0f;

    oldTip->left_end->u = u_rest - epsilon_u;
    oldTip->right_end = vRight;

    // 关键：使用正确的第 i 层边界同步
    syncPositions(cn.size() - 1, vL_i, vR_i);

    // 2. 创建新尖端 (处于 level_i + 1)
    int nextLevel = oldTip->level + 1;
    glm::vec3 posNext = (1.0f - u_rest) * vL_next->position + u_rest * vR_next->position;

    auto vNewShared = m_pool->createVertex(u_rest, posNext);
    vNewShared->prevPosition = posNext;
    vNewShared->invMass = 10.0f;

    if (nextLevel == MAX_LAYER - 1)
        vNewShared->invMass = 0.0f;


    cn.push_back(std::make_unique<CrackNode>(nextLevel, vNewShared, vNewShared));

    return true;
}

// 向下合并：基于你的原则，如果只剩两层，合并时全部消失
bool Crack::goDownOnce() {
    if (cn.size() <= 2) {
        // 如果只有 Level 0 和 Level 1，直接清空，让裂缝彻底消失
        cn.clear();
        return true;
    }

    // 如果多于两层：
    // 1. 移除最顶层的共享尖端
    cn.pop_back();

    // 2. 将现在的最顶层（原分裂层）合并为共享点
    CrackNode* newTip = cn.back().get();
    glm::vec3 avgPos = (newTip->left_end->position + newTip->right_end->position) * 0.5f;
    glm::vec3 avgPrevPos = (newTip->left_end->prevPosition + newTip->right_end->prevPosition) * 0.5f;

    newTip->left_end->position = avgPos;
    newTip->left_end->prevPosition = avgPrevPos;
    newTip->right_end = newTip->left_end;

    return true;
}

void Crack::syncPositions(int level, std::shared_ptr<Vertex> vL, std::shared_ptr<Vertex> vR) {
    if (level >= cn.size()) return;

    auto& node = cn[level];

    if (node->left_end == node->right_end) {
        node->left_end->u = u_rest;
        node->left_end->position = (1.0f - u_rest) * vL->position + u_rest * vR->position;
        node->left_end->prevPosition = node->left_end->position;
        return;
    }

    // --- 强制几何拓扑一致性 ---
    // 1. 确保左侧不越过中线，右侧不越过中线
    node->left_end->u = std::min(node->left_end->u, u_rest - 1e-5f);
    node->right_end->u = std::max(node->right_end->u, u_rest + 1e-5f);

    // 2. 确保不超出 Bundle 边界 [0, 1]
    node->left_end->u = std::max(node->left_end->u, 0.001f);
    node->right_end->u = std::min(node->right_end->u, 0.999f);

    float uL = node->left_end->u;
    float uR = node->right_end->u;

    // 2. 计算基准线上的投影位置 (LERP)
    // 这样无论 Mesh 如何倾斜，顶点永远在 L-R 连线上
    node->left_end->position = (1.0f - uL) * vL->position + uL * vR->position;
    node->right_end->position = (1.0f - uR) * vL->position + uR * vR->position;

    // 3. 关键：同步 prevPosition 以消除纵向抖动产生的虚假动能
    node->left_end->prevPosition = node->left_end->position;
    node->right_end->prevPosition = node->right_end->position;
}
