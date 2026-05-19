#ifndef HAIR2D_BUNDLE_H
#define HAIR2D_BUNDLE_H

#include<map>
#include "Shader.h"
#include "GeometryHelpers.h"
#include "Crack.h"
#include "VPool.h"


class Bundle {
public:
    VPool* vpool{nullptr};

    int num_layers;
    int num_strands;
    std::map<float, Crack> cracks;

    // 左右边缘点
    std::vector<std::shared_ptr<Vertex>> left_vertices;
    std::vector<std::shared_ptr<Vertex>> right_vertices;

    // 端点+裂缝点
    std::vector<std::vector<std::shared_ptr<Vertex>>> vtx_matrix;

    std::vector<std::vector<float>> L0;

    float step;

    // Rendering Data
    Shader* f_shader{nullptr}; // Frame Shader
    Shader* s_shader{nullptr}; // Strand Shader

    unsigned int VAO1, VAO2, VBO1, VBO2;

    // 渲染参数
    glm::vec3 frame_color{glm::vec3(0.0f, 0.0f, 0.0f)}; // 黑色网格框
    float frame_thickness{0.01f};
    float strand_thickness{0.006f};

    // 顶点数据缓存
    std::vector<float> fatline_frame_vertices;          // 网格框架 (Frame)
    std::vector<float> fatline_strands_vertices; // 内部发丝 (Strands)

    // 构造函数：接收初始位置数组和发丝数
    Bundle(VPool* vp, const std::vector<glm::vec3>& left_corners, const std::vector<glm::vec3>& righ_corners, int num_strands);
    ~Bundle();

    // 生成每一帧的渲染数据 (遍历树)
    void initRenderData();

    void refreshVtxMatrix();
    // 绘制调用
    void draw();


};


#endif
