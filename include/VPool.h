#ifndef HAIR2D_VPOOL_H
#define HAIR2D_VPOOL_H

#include <unordered_map>
#include <memory>
#include <queue>
#include "Vertex.h"


class VPool {
public:
    VPool();
    ~VPool(); // 需要在 cpp 中实现

    std::shared_ptr<Vertex> createVertex(float u_rest, const glm::vec3& position);

    // 标记为可复用，但不立即释放内存，留给下次 createVertex 复用
    void deleteVertex(int id);

    std::shared_ptr<Vertex> getVertex(int id) const;

    size_t getActiveVertexCount() const; // 当前活跃的顶点数
    size_t getPoolSize() const;          // 池子总容量（活跃+闲置）

    void clearAll();

private:
    std::unordered_map<int, std::shared_ptr<Vertex>> m_pool;
    std::queue<int> reusable_ids;
    int next_id;
};


#endif //HAIR2D_VPOOL_H