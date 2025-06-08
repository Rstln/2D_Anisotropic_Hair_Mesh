#include "Bundle.h"

Bundle::Bundle(std::vector<std::array<Vertex, 2>>&& l, unsigned int n) :
        layers(std::move(l)), num_strands(n), step(1.0f / static_cast<float>(num_strands+1))
{
    L0.resize(layers.size(), std::vector<float>(num_strands, 0.0f));

    flatten.resize((3*layers.size() - 2) * 2 * 3 * 2, 0.0f);
    flatten_strands.resize(num_strands * (layers.size()-1) * 3 * 2, 0.0f);

    // init_layers(layers);
    // cal L0
    for (int i = 1; i < layers.size(); i++) {
        float coeff = 0.0f;
        for (int j = 0; j < num_strands; j++) {
            coeff += step;
            auto tmp1 = coeff * layers[i-1][1].position + (1-coeff) * layers[i-1][0].position;
            auto tmp2 = coeff * layers[i][1].position + (1-coeff) * layers[i][0].position;
            L0[i][j] = glm::length(tmp1-tmp2);
        }
    }

    // set fixed points manually
    for (int i = 0; i < 1; i++) {
        for (auto& v : layers[i]) {
            v.invMass = 0.0f;
        }
    }


    // OpenGL rendering
    glGenVertexArrays(1, &VAO1);
    glGenVertexArrays(1, &VAO2);
    glGenBuffers(1, &VBO1);
    glGenBuffers(1, &VBO2);

    glBindVertexArray(VAO1);
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, flatten.size() * sizeof(float), flatten.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*) nullptr);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glBindVertexArray(VAO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, flatten_strands.size() * sizeof(float), flatten_strands.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*) nullptr);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}


void Bundle::pushFlatten(std::vector<float> &f, glm::vec3 pos, int idx) {
    f[idx++] = pos.x;
    f[idx++] = pos.y;
    f[idx++] = pos.z;
}


void Bundle::initRenderData() {
    int idx = 0, idx_s = 0;

    // flatten rot layer pos data
    pushFlatten(flatten, layers[0][0].position, idx);
    idx += 3;
    pushFlatten(flatten, layers[0][1].position, idx);
    idx += 3;

    for (int i = 1; i < layers.size(); i++) {
        // flatten pos data
        pushFlatten(flatten, layers[i-1][0].position, idx);
        idx += 3;
        pushFlatten(flatten, layers[i][0].position, idx);
        idx += 3;

        pushFlatten(flatten, layers[i-1][1].position, idx);
        idx += 3;
        pushFlatten(flatten, layers[i][1].position, idx);
        idx += 3;

        pushFlatten(flatten, layers[i][0].position, idx);
        idx += 3;
        pushFlatten(flatten, layers[i][1].position, idx);
        idx += 3;

        // flatten strands data
        float coeff = 0.0f;
        for (int j = 0; j < num_strands; j++) {
            coeff += step;
            auto tmp1 = coeff * layers[i-1][1].position + (1-coeff) * layers[i-1][0].position;
            auto tmp2 = coeff * layers[i][1].position + (1-coeff) * layers[i][0].position;
            pushFlatten(flatten_strands, tmp1, idx_s);
            idx_s += 3;
            pushFlatten(flatten_strands, tmp2, idx_s);
            idx_s += 3;
        }
    }


    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferSubData(GL_ARRAY_BUFFER, 0, flatten.size() * sizeof(float), flatten.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferSubData(GL_ARRAY_BUFFER, 0, flatten_strands.size() * sizeof(float), flatten_strands.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void Bundle::draw() {
    initRenderData();

    fShader->use();

    glBindVertexArray(VAO1);
    glDrawArrays(GL_LINES, 0, flatten.size() / 3);
    glBindVertexArray(0);

    hShader->use();

    glBindVertexArray(VAO2);
    glDrawArrays(GL_LINES, 0, flatten_strands.size() / 3);
    glBindVertexArray(0);
}