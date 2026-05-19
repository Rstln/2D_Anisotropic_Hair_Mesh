#include "VPool.h"

VPool::VPool() : next_id(0) {

}

VPool::~VPool() {
    clearAll();
}

std::shared_ptr<Vertex> VPool::createVertex(float u_rest, const glm::vec3 &position) {
    // 1. 尝试复用
    if (!reusable_ids.empty()) {
        int id = reusable_ids.front();
        reusable_ids.pop();

        auto it = m_pool.find(id);
        if (it != m_pool.end() && it->second) {
            // [关键优化] 复用内存：重置现有对象的状态
            auto& vtx = it->second;
            vtx->u = u_rest;
            vtx->u_rest = u_rest;
            vtx->position = position;
            vtx->prevPosition = position; // 初始时 prev = curr，防止初速度爆炸
            vtx->velocity = glm::vec3(0.0f);
            vtx->normal = glm::vec3(0.0f, 1.0f, 0.0f); // 重置默认法线
            vtx->invMass = 1.0f; // 重置质量 (ATree::split 会随后覆盖这个值)

            // 注意：id 还是原来的 id，不需要变
            return vtx;
        }
    }

    // 2. 如果没有可复用的，才创建新的
    int id = next_id++;
    auto vtx = std::make_shared<Vertex>(u_rest, position);
    m_pool[id] = vtx;

    return vtx;
}

void VPool::deleteVertex(int id) {
    // 只是将 ID 放回队列，并不从 map 中 erase
    // 这样 shared_ptr 还在，内存还在，下次可以直接复用
    auto it = m_pool.find(id);
    if (it != m_pool.end()) {
        reusable_ids.push(id);
    }
}

std::shared_ptr<Vertex> VPool::getVertex(int id) const {
    auto it = m_pool.find(id);
    return (it != m_pool.end()) ? it->second : nullptr;
}

void VPool::clearAll() {
    m_pool.clear(); // 这会释放所有 shared_ptr，真正释放 Vertex 内存
    while (!reusable_ids.empty()) {
        reusable_ids.pop();
    }
    next_id = 0;
}

size_t VPool::getActiveVertexCount() const {
    return m_pool.size() - reusable_ids.size();
}

size_t VPool::getPoolSize() const {
    return m_pool.size();
}
