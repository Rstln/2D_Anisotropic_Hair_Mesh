#ifndef HAIR2D_BUNDLE_H
#define HAIR2D_BUNDLE_H


#include "Vertex.h"
#include "Shader.h"
#include <cmath>


class Bundle {
public:
    // 0th layer is root layer, fixed
    std::vector<std::array<Vertex, 2>> layers;
    // std::vector<std::array<Vertex, 2>> init_pos;
    // num_layers * num_strands
    std::vector<std::vector<float>> L0;
    // std::vector<float> flatten;
    std::vector<std::array<glm::vec3, 2>> strands;

    int num_strands;
    float step;

    // rendering
    Shader* hShader{nullptr};
    Shader* fShader{nullptr};
    unsigned int VAO1, VAO2, VBO1, VBO2;
    std::vector<float> flatten;
    std::vector<float> flatten_strands;

    explicit Bundle(std::vector<std::array<Vertex, 2>>&& l, unsigned int n);

    static inline void pushFlatten(std::vector<float>& f, glm::vec3 pos, int idx);

    void initRenderData();

    void draw();
};




#endif
