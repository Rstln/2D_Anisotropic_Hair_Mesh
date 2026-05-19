#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <algorithm>

// OpenGL includes
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// GLM includes
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Project includes
#include "XPBDSimulator.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ================================= global parameter =================================

const unsigned int SCR_WIDTH = 600;
const unsigned int SCR_HEIGHT = 600;
const float FIXED_DT = 1.0f / 500.0f;
const int MAX_STEPS = 200;

// ================================= bundle data =================================
std::vector<glm::vec3> left_corners = {
    glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(-0.5f, -0.25f, 0.0f),
    glm::vec3(-0.5f, 0.0f, 0.0f),  glm::vec3(-0.5f, 0.25f, 0.0f),
    glm::vec3(-0.5f, 0.5f, 0.0f),
};

std::vector<glm::vec3> right_corners = {
    glm::vec3(0.5f, -0.5f, 0.0f), glm::vec3(0.5f, -0.25f, 0.0f),
    glm::vec3(0.5f, 0.0f, 0.0f),  glm::vec3(0.5f, 0.25f, 0.0f),
    glm::vec3(0.5f, 0.5f, 0.0f),
};

// ================================= aux functions =================================
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
bool saveFramebufferPNG(const std::string& path, int width, int height);

int main(int argc, char** argv) {
    bool captureMode = false;
    std::string capturePath = "../output/preview.png";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--capture") {
            captureMode = true;
        } else if (arg == "--capture-path" && i + 1 < argc) {
            captureMode = true;
            capturePath = argv[++i];
        }
    }

    // 1. Init GLFW
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Two Colliders Test", nullptr, nullptr);
    if (window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // 2. Init GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // ================= Init Data =================
    VPool vpool;
    Bundle bd1(&vpool, left_corners, right_corners, 7);

    // [修改 1] 定义两个 Collider
    // ---------------------------------------------------------
    // 左边的大球: x=-0.2, 半径=0.1
    glm::vec3 posLeft(-0.2f, -0.8f, 0.0f);
    Collider colLeft(posLeft, 0.1f);

    // 右边的小球: x=0.3, 半径=0.05
    glm::vec3 posRight(0.3f, -0.7f, 0.0f);
    Collider colRight(posRight, 0.07f);

    // 加入模拟器
    std::vector<Collider*> colliders = { &colLeft, &colRight };
    XPBDSimulator simulator(&bd1, colliders);

    // ================= Init Shaders =================
    Shader frame_shader("../shaders/shader.vert", "../shaders/shader.frag");
    bd1.f_shader = &frame_shader;

    Shader strand_shader("../shaders/vis_shader.vert", "../shaders/vis_shader.frag");
    bd1.s_shader = &strand_shader;

    Shader collider_shader("../shaders/collider_shader.vert", "../shaders/shader.frag");
    colLeft.m_shader = &collider_shader;
    colRight.m_shader = &collider_shader;

    // ================= 统计输出 & 动画定义 =================
    std::ofstream csvFile("../output/stats_two_colliders.txt");
    if (csvFile.is_open()) {
        csvFile << "Time,Sum_C_Length_Sq,Sum_C_Angle_Sq\n";
    }

    // 动画控制 Lambda
    auto updateColliderUpward = [](Collider& col, glm::vec3 basePos, float jumpHeight,
                                   float duration, float startTime, float currentTime) {
        float elapsed = currentTime - startTime;
        glm::vec3 newPos = basePos;

        // 在持续时间内执行正弦波运动 (0 -> 1 -> 0)
        if (elapsed > 0.0f && elapsed < duration) {
            float angle = (elapsed / duration) * 3.1415926f; // map to [0, PI]
            float yOffset = jumpHeight * std::sin(angle);
            newPos.y += yOffset;
        }
        col.setPosition(newPos);
    };

    // 动画参数配置
    float animHeight = 1.0f;
    float animDuration = 8.0f;
    float animStartTime = 1.0f;

    float lastTime = glfwGetTime();
    float accumulator = 0.0f;
    bool captured = false;
    bool capturePrepared = false;

    // ================= Rendering Loop =================
    while(!glfwWindowShouldClose(window)) {
        float currentTime = glfwGetTime();
        float frameTime = currentTime - lastTime;
        lastTime = currentTime;
        if (frameTime > 0.25f) frameTime = 0.25f;
        accumulator += frameTime;

        processInput(window);

        if (captureMode && !capturePrepared) {
            const int warmupSteps = 450;
            for (int i = 0; i < warmupSteps; ++i) {
                float simTime = i * FIXED_DT;
                updateColliderUpward(colLeft, posLeft, animHeight, animDuration, animStartTime, simTime);
                updateColliderUpward(colRight, posRight, animHeight, animDuration, animStartTime, simTime);
                simulator.substep();
            }
            currentTime = warmupSteps * FIXED_DT;
            capturePrepared = true;
        }

        glClearColor(0.95f, 0.95f, 0.95f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Projection
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        float aspectRatio = (width > 0 && height > 0) ? (float)width / height : 1.0f;
        glm::mat4 projection = glm::ortho(-aspectRatio, aspectRatio, -1.0f, 1.0f, -1.0f, 1.0f);

        // [修改 2] 更新两个 Collider 的位置
        updateColliderUpward(colLeft, posLeft, animHeight, animDuration, animStartTime, currentTime);
        updateColliderUpward(colRight, posRight, animHeight, animDuration, animStartTime, currentTime);

        // 绘制 Colliders
        colLeft.draw(projection);
        colRight.draw(projection);

        // Physics Step
        if (!captureMode) {
            int stepCount = 0;
            while (accumulator >= FIXED_DT && stepCount < MAX_STEPS) {
                simulator.substep();
                accumulator -= FIXED_DT;
                stepCount++;
            }
        }

        // ================= 统计与显示 =================
        // float lenErr = simulator.computeTotalLengthCSq();
        // float angErr = simulator.computeTotalAngleCSq();
        //
        // // 写文件
        // if (csvFile.is_open()) {
        //     csvFile << currentTime << "," << lenErr << "," << angErr << "\n";
        // }
        //
        // // 更新标题栏
        // std::stringstream ss;
        // ss << "Time: " << std::fixed << std::setprecision(2) << currentTime << "s"
        //    << " | LenErr: " << std::setprecision(5) << lenErr
        //    << " | AngErr: " << std::setprecision(5) << angErr;
        // glfwSetWindowTitle(window, ss.str().c_str());

        // Draw Hair
        bd1.f_shader->use();
        bd1.f_shader->setMat4("u_projection", projection);
        bd1.s_shader->use();
        bd1.s_shader->setMat4("u_projection", projection);
        bd1.draw();

        if (captureMode && !captured) {
            if (saveFramebufferPNG(capturePath, width, height)) {
                std::cout << "Saved capture to " << capturePath << std::endl;
            } else {
                std::cout << "Failed to save capture to " << capturePath << std::endl;
            }
            captured = true;
            glfwSetWindowShouldClose(window, true);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    if (csvFile.is_open()) csvFile.close();
    glfwTerminate();
    return 0;
}

// Aux functions
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}
void processInput(GLFWwindow *window) {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

bool saveFramebufferPNG(const std::string& path, int width, int height) {
    if (width <= 0 || height <= 0) return false;

    std::filesystem::path outPath(path);
    if (!outPath.parent_path().empty()) {
        std::filesystem::create_directories(outPath.parent_path());
    }

    std::vector<unsigned char> pixels(width * height * 3);
    std::vector<unsigned char> flipped(width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    const int stride = width * 3;
    for (int y = 0; y < height; ++y) {
        std::copy_n(&pixels[(height - 1 - y) * stride], stride, &flipped[y * stride]);
    }

    return stbi_write_png(path.c_str(), width, height, 3, flipped.data(), stride) != 0;
}
