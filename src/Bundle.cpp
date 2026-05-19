#include "Bundle.h"

using namespace GeometryHelpers;


Bundle::Bundle(VPool* vp, const std::vector<glm::vec3> &left_corners, const std::vector<glm::vec3> &right_corners, int num_strands)
    :vpool(vp), num_strands(num_strands)
{
    left_vertices.clear();
    right_vertices.clear();
    num_layers = left_corners.size();
    vtx_matrix.clear();
    vtx_matrix.resize(num_layers);

    for (int i = 0; i < num_layers; i++) {
        left_vertices.push_back(vpool->createVertex(0.0f, left_corners[i]));
        right_vertices.push_back(vpool->createVertex(1.0f, right_corners[i]));
        // left_vertices.back()->invMass = 1.0f / (i+1);
    }
    // 0th layer is fixed, invMass = 0.0f
    left_vertices.back()->invMass = 0.0f;
    right_vertices.back()->invMass = 0.0f;


    // 计算L0
    step = 1.0f / (num_strands - 1);
    L0.resize(num_layers-1);
    for (int i = 0; i < num_layers - 1; i++) {
        for (int j = 0; j < num_strands; j++) {
            auto p_down = j * step * right_vertices[i]->position + (num_strands-1-j) * step * left_vertices[i]->position;
            auto p_up = j * step * right_vertices[i+1]->position + (num_strands-1-j) * step * left_vertices[i+1]->position;
            L0[i].push_back(glm::length(p_up - p_down));
        }
    }


    // OpenGL rendering setup for fatline_frame
    glGenVertexArrays(1, &VAO1);
    glGenBuffers(1, &VBO1);

    glBindVertexArray(VAO1);
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)nullptr);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glGenVertexArrays(1, &VAO2);
    glGenBuffers(1, &VBO2);

    glBindVertexArray(VAO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)nullptr);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    refreshVtxMatrix();
}

Bundle::~Bundle() {
    if (VAO1 != 0) glDeleteVertexArrays(1, &VAO1);
    if (VAO2 != 0) glDeleteVertexArrays(1, &VAO2);
    if (VBO1 != 0) glDeleteBuffers(1, &VBO1);
    if (VBO2 != 0) glDeleteBuffers(1, &VBO2);
}


void Bundle::initRenderData() {
    fatline_frame_vertices.clear();
    fatline_strands_vertices.clear();

    refreshVtxMatrix();

    // =====================
    // 添加mesh边框 frame
    // =====================
    // 左边纵线
    for (int i = 0; i < num_layers; i++) {
        if (i < num_layers - 1) {
            // 添加纵向线
            auto p3 = left_vertices[i]->position;
            auto p4 = left_vertices[i + 1]->position;
            addQuad(fatline_frame_vertices, p3, p4, frame_thickness);
        }
    }

    // 从下往上塞，添加裂缝，纵线
    for (auto&[fst, snd] : cracks) {
        for (int i = 0; i < snd.cn.size(); i++) {
            if (i < snd.cn.size() - 1) {
                addQuad(fatline_frame_vertices, snd.cn[i]->left_end->position, snd.cn[i+1]->left_end->position, frame_thickness);
                addQuad(fatline_frame_vertices, snd.cn[i]->right_end->position, snd.cn[i+1]->right_end->position, frame_thickness);
            }
        }
    }
    // 右边纵线
    for (int i = 0; i < num_layers; i++) {
       if (i < num_layers - 1) {
           auto p3 = right_vertices[i]->position;
           auto p4 = right_vertices[i + 1]->position;
           addQuad(fatline_frame_vertices, p3, p4, frame_thickness);
       }
    }

    // 添加横向线
    for (int i = 0; i < num_layers; i++) {
        for (int j = 0; j < vtx_matrix[i].size(); j+=2) {
            auto p1 = vtx_matrix[i][j]->position;
            auto p2 = vtx_matrix[i][j+1]->position;
            addQuad(fatline_frame_vertices, p1, p2, frame_thickness);
        }
    }

    // =====================
    // 添加发丝strand
    // =====================
    for (int i = 0; i < num_layers - 1; i++) {
        // 1. 定义获取有序断点的 Lambda 函数
        auto get_sorted_breakpoints = [&](int layer_idx) {
            struct BreakPoint {
                float u;
                std::shared_ptr<Vertex> v_left_of_gap;
                std::shared_ptr<Vertex> v_right_of_gap;
            };
            std::vector<BreakPoint> pts;
            pts.push_back({ 0.0f, nullptr, left_vertices[layer_idx] });
            pts.push_back({ 1.0f, right_vertices[layer_idx], nullptr });

            for (auto& [u_val, crack] : cracks) {
                for (auto& node : crack.cn) {
                    if (node->level == layer_idx) {
                        pts.push_back({ u_val, node->left_end, node->right_end });
                        break;
                    }
                }
            }
            std::sort(pts.begin(), pts.end(), [](const BreakPoint& a, const BreakPoint& b) {
                return a.u < b.u;
            });
            return pts;
        };

        // 直接使用 auto 接收返回的 vector
        auto bps_up = get_sorted_breakpoints(i);
        auto bps_down = get_sorted_breakpoints(i + 1);

        // 2. 遍历发丝，跳过 j=0 (u=0) 和 j=num_strands-1 (u=1)
        for (int j = 1; j < num_strands-1; j++) {
            float u = j * step;

            // --- 寻找上层控制区间 ---
            std::shared_ptr<Vertex> pUL, pUR;
            float uUL = 0.0f, uUR = 1.0f;
            for (int k = 0; k < (int)bps_up.size() - 1; k++) {
                if (u >= bps_up[k].u && u <= bps_up[k+1].u) {
                    pUL = bps_up[k].v_right_of_gap;
                    pUR = bps_up[k+1].v_left_of_gap;
                    uUL = bps_up[k].u; uUR = bps_up[k+1].u;
                    break;
                }
            }

            // --- 寻找下层控制区间 ---
            std::shared_ptr<Vertex> pDL, pDR;
            float uDL = 0.0f, uDR = 1.0f;
            for (int k = 0; k < (int)bps_down.size() - 1; k++) {
                if (u >= bps_down[k].u && u <= bps_down[k+1].u) {
                    pDL = bps_down[k].v_right_of_gap;
                    pDR = bps_down[k+1].v_left_of_gap;
                    uDL = bps_down[k].u; uDR = bps_down[k+1].u;
                    break;
                }
            }

            if (pUL && pUR && pDL && pDR) {
                // 3. 计算局部插值比例并得到端点位置
                float t_up = (uUR - uUL > 1e-7f) ? (u - uUL) / (uUR - uUL) : 0.0f;
                float t_down = (uDR - uDL > 1e-7f) ? (u - uDL) / (uDR - uDL) : 0.0f;

                glm::vec3 pos_up = (1.0f - t_up) * pUL->position + t_up * pUR->position;
                glm::vec3 pos_down = (1.0f - t_down) * pDL->position + t_down * pDR->position;

                // 4. 计算长度偏差 diff 供热力图显示
                float current_len = glm::distance(pos_up, pos_down);
                float rest_length = L0[i][j];
                float diff = current_len - rest_length;

                // 5. 调用带颜色的 Quad 绘制
                addColoredQuad(fatline_strands_vertices, pos_up, pos_down, strand_thickness, get_log_jet_color(diff));
            }
        }
    }

    // 更新 VBO
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, fatline_frame_vertices.size() * sizeof(float), fatline_frame_vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, fatline_strands_vertices.size() * sizeof(float), fatline_strands_vertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void Bundle::refreshVtxMatrix() {
    // 1. 确保矩阵的层级数量正确
    if (vtx_matrix.size() != num_layers) {
        vtx_matrix.resize(num_layers);
    }

    // 2. 逐层刷新顶点顺序
    for (int i = 0; i < num_layers; i++) {
        // 清空当前层的旧数据，准备重新填充
        vtx_matrix[i].clear();

        // A. 首先插入左边缘顶点 (u = 0.0)
        vtx_matrix[i].push_back(left_vertices[i]);

        // B. 收集并排序这一层所有的裂缝节点
        // 因为同一层可能有多个裂缝，必须按 u_rest 从左到右排序
        struct LayerNode {
            float u;
            std::shared_ptr<Vertex> left;
            std::shared_ptr<Vertex> right;
        };
        std::vector<LayerNode> activeCracks;

        for (auto& [u_val, crack] : cracks) {
            // 查找该裂缝是否覆盖到了当前层 i
            // 由于裂缝是连续的，我们只需遍历该 crack 的 cn 数组
            for (auto& node : crack.cn) {
                if (node->level == i) {
                    activeCracks.push_back({ u_val, node->left_end, node->right_end });
                    break;
                }
            }
        }

        // 按 u 坐标升序排序，确保拓扑顺序正确
        std::sort(activeCracks.begin(), activeCracks.end(),
            [](const LayerNode& a, const LayerNode& b) {
                return a.u < b.u;
            }
        );

        // C. 将排序后的裂缝顶点对插入矩阵
        for (auto& ln : activeCracks) {
            // 即使是共享尖端（两个指针指向同一个实体），也要存入两次
            // 这样保证了矩阵每一行的结构都是：[左, 裂缝1L, 裂缝1R, 裂缝2L, 裂缝2R, ..., 右]
            vtx_matrix[i].push_back(ln.left);
            vtx_matrix[i].push_back(ln.right);
        }

        // D. 最后插入右边缘顶点 (u = 1.0)
        vtx_matrix[i].push_back(right_vertices[i]);
    }
}



void Bundle::draw() {
    initRenderData();

    if (f_shader) {
        f_shader->use();
        f_shader->setVec3("lineColor", frame_color);
        // 注意：投影矩阵需要在外部或此处设置
        glBindVertexArray(VAO1);
        glDrawArrays(GL_TRIANGLES, 0, fatline_frame_vertices.size() / 3);
        glBindVertexArray(0);
    }

    if (s_shader) {
        s_shader->use();
        // s_shader->setVec3("lineColor", strand_color); // 使用顶点颜色
        glBindVertexArray(VAO2);
        glDrawArrays(GL_TRIANGLES, 0, fatline_strands_vertices.size() / 6);
        glBindVertexArray(0);
    }
}
