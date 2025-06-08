cmake_minimum_required(VERSION 3.31)
project(hair2d)

set(CMAKE_CXX_STANDARD 17)

include_directories(./external/glad/include)

include_directories(/opt/homebrew/Cellar/glfw/3.4/include)

include_directories(/opt/homebrew/Cellar/glm/1.0.1/include/)

include_directories(./include/)

link_directories(/opt/homebrew/Cellar/glfw/3.4/lib)



add_executable(hair2d external/glad/src/glad.c
        src/main.cpp
        include/Shader.h
        src/Shader.cpp
        include/Collider.h
        src/Collider.cpp
        include/Vertex.h
        src/Vertex.cpp
        include/XPBDSimulator.h
        include/Bundle.h
        src/Bundle.cpp
        src/XPBDSimulator.cpp
)


target_link_libraries(hair2d glfw)
