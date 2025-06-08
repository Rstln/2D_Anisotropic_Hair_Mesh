#ifndef HAIR2D_SHADER_H
#define HAIR2D_SHADER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>


class Shader {
public:
    unsigned int ID;
    mutable std::unordered_map<std::string, int> location_cache;

    Shader(const char* vertexPath, const char* fragmentPath, const char* geometryPath = nullptr);
    void use();
    void setBool(const std::string& name, bool value) const;
    void setInt(const std::string& name, int value) const;
    void setFloat(const std::string& name, float value) const;
    void setMat4(const std::string& name, glm::mat4 value) const;
    void setVec2(const std::string& name, glm::vec2 value) const;
    void setVec3(const std::string& name, glm::vec3 value) const;
    void setVec4(const std::string& name, glm::vec4 value) const;
    int GetUniformLocation(const std::string& name) const;
private:
    char* ReadSource(const char* filename);
    void CheckCompileError(unsigned int shader, std::string type);
};


#endif
