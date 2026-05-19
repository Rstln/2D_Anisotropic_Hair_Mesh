#ifndef HAIR2D_CRACK_H
#define HAIR2D_CRACK_H

#include "CrackNode.h"
#include "VPool.h"

class Crack {
public:
    int MAX_LAYER;
    const float epsilon_u{0.1f};
    const float k_restore{0.0001};
    int collision_layer{-1};
    float u_rest;
    std::vector<std::unique_ptr<CrackNode>> cn;
    VPool* m_pool{nullptr}; // 存储指向顶点池的指针

    // 构造函数：现在需要接收 VPool 指针
    Crack(float u_rest, VPool* pool,
             std::shared_ptr<Vertex> vL_bound0, std::shared_ptr<Vertex> vR_bound0,
             std::shared_ptr<Vertex> vL_bound1, std::shared_ptr<Vertex> vR_bound1,
             int MAX_LAYER);

    // 延伸函数：不再需要传入 pool
    bool goUpOnce(std::shared_ptr<Vertex> vL_i, std::shared_ptr<Vertex> vR_i,
                     std::shared_ptr<Vertex> vL_next, std::shared_ptr<Vertex> vR_next);

    void syncPositions(int level, std::shared_ptr<Vertex> vL_bound, std::shared_ptr<Vertex> vR_bound);
    bool goDownOnce();
    void clearCrack();
};


#endif //HAIR2D_CRACK_H