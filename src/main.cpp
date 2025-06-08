#include <iostream>
#include "XPBDSimulator.h"

// ================================= global parameter =================================

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;
Collider* mouseCollider = nullptr;

// ================================= bundle data =================================
std::vector<std::array<Vertex, 2>> bd_data() {
    std::array<Vertex, 2> l1 = {
            Vertex(glm::vec3(-0.5f, 0.5f, 0.0f)),
            Vertex(glm::vec3(0.5f, 0.5f, 0.0f)),
    };

    std::array<Vertex, 2> l2 = {
            Vertex(glm::vec3(-0.5f, 0.25f, 0.0f)),
            Vertex(glm::vec3(0.5f, 0.25f, 0.0f)),
    };

    std::array<Vertex, 2> l3 = {
            Vertex(glm::vec3(-0.5f, -0.0f, 0.0f)),
            Vertex(glm::vec3(0.5f, -0.0f, 0.0f)),
    };

    std::array<Vertex, 2> l4 = {
            Vertex(glm::vec3(-0.5f, -0.25f, 0.0f)),
            Vertex(glm::vec3(0.5f, -0.25f, 0.0f)),
    };

    std::array<Vertex, 2> l5 = {
            Vertex(glm::vec3(-0.5f, -0.5f, 0.0f)),
            Vertex(glm::vec3(0.5f, -0.5f, 0.0f)),
    };

    std::vector<std::array<Vertex, 2>> lys;
    lys.push_back(std::move(l1));
    lys.push_back(std::move(l2));
    lys.push_back(std::move(l3));
    lys.push_back(std::move(l4));
    lys.push_back(std::move(l5));
    return lys;
}


// ================================= aux functions for OpenGL =================================
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

void DrawDebugGrid() {
    // 使用 static 变量，确保所有资源只被创建和初始化一次。
    static GLuint gridProgramID = 0;
    static GLuint gridVAO = 0;
    static int vertexCount = 0;

    // 检查 programID 是否为0。如果为0，说明是第一次调用此函数，需要初始化所有资源。
    if (gridProgramID == 0) {
        // 1. --- 一次性设置：编译着色器 ---
        const char* vertexSource = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            void main() {
                gl_Position = vec4(aPos, 1.0);
            }
        )";

        const char* fragmentSource = R"(
            #version 330 core
            out vec4 FragColor;
            void main() {
                // 颜色直接硬编码为蓝色
                FragColor = vec4(0.2f, 0.4f, 0.8f, 1.0f);
            }
        )";

        // -- 编译过程 --
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &vertexSource, NULL);
        glCompileShader(vs);
        // 简单错误检查
        int success;
        char infoLog[512];
        glGetShaderiv(vs, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(vs, 512, NULL, infoLog);
            std::cerr << "ERROR::GRID_SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fragmentSource, NULL);
        glCompileShader(fs);
        glGetShaderiv(fs, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(fs, 512, NULL, infoLog);
            std::cerr << "ERROR::GRID_SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        gridProgramID = glCreateProgram();
        glAttachShader(gridProgramID, vs);
        glAttachShader(gridProgramID, fs);
        glLinkProgram(gridProgramID);
        glGetProgramiv(gridProgramID, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(gridProgramID, 512, NULL, infoLog);
            std::cerr << "ERROR::GRID_SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }
        glDeleteShader(vs);
        glDeleteShader(fs);


        // 2. --- 一次性设置：创建网格顶点数据和缓冲区 ---
        std::vector<glm::vec3> vertices;
        float step = 0.1f;
        const float z_depth = 0.999f; // 确保在背景

        for (float i = -1.0f; i <= 1.0f; i += step) {
            vertices.push_back(glm::vec3(i, -1.0f, z_depth));
            vertices.push_back(glm::vec3(i,  1.0f, z_depth));
            vertices.push_back(glm::vec3(-1.0f, i, z_depth));
            vertices.push_back(glm::vec3( 1.0f, i, z_depth));
        }
        vertexCount = vertices.size();

        GLuint gridVBO = 0;
        glGenVertexArrays(1, &gridVAO);
        glGenBuffers(1, &gridVBO);
        glBindVertexArray(gridVAO);
        glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    // --- 每帧执行的绘制部分 ---
    glUseProgram(gridProgramID);
    glBindVertexArray(gridVAO);
    glDrawArrays(GL_LINES, 0, vertexCount);
    glBindVertexArray(0);
}



int main() {
    // ================================= init glfw =================================
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "hair2d", nullptr, nullptr);
    if (window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // ================================= init glad =================================
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }


    // ================================= init data =================================
    auto layers1 = bd_data();
    Bundle bd1(std::move(layers1), 5);


    Collider local_collider(glm::vec3(0.0f, -0.85f, 0.0f), 0.1f);
    mouseCollider = &local_collider;

    XPBDSimulator simulator(&bd1, &local_collider);


    // ================================= init shaders =================================
    Shader strand_shader("../shaders/shader.vert", "../shaders/shader.frag");
    Shader frame_shader("../shaders/shader.vert", "../shaders/shader_b.frag");
    bd1.hShader = &strand_shader;
    bd1.fShader = &frame_shader;

    Shader collider_shader("../shaders/collider_shader.vert", "../shaders/collider_shader.frag");
    local_collider.m_shader = &collider_shader;

    // ================================= initial state =================================
    //simulator.addVel();

    // ================================= rendering loop =================================
    while(!glfwWindowShouldClose(window)) {
        processInput(window);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);


        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        float aspectRatio = (width > 0 && height > 0) ? (float)width / (float)height : 1.0f;
        glm::mat4 projection = glm::ortho(-aspectRatio, aspectRatio, -1.0f, 1.0f, -1.0f, 1.0f);
        if (mouseCollider) {
            mouseCollider->draw(projection);
        }

        simulator.substep();
        bd1.draw();

        // swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mouseCollider = nullptr;
    glfwTerminate();

    return 0;
}


void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window) {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}


void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    float aspectRatio = (width > 0 && height > 0) ? (float)width / (float)height : 1.0f;

    // 将鼠标屏幕坐标转换为 [-aspectRatio, aspectRatio] x [-1, 1]
    float norm_x = (float)xpos / width;     // [0, 1]
    float norm_y = (float)ypos / height;    // [0, 1]

    float world_x = norm_x * 2.0f * aspectRatio - aspectRatio;
    float world_y = (1.0f - norm_y) * 2.0f - 1.0f;

    mouseCollider->setPosition(glm::vec3(world_x, world_y, 0.0f));
}