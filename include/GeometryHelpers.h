#ifndef HAIR2D_GEOMETRYHELPERS_H
#define HAIR2D_GEOMETRYHELPERS_H

#include <glm/glm.hpp>
#include <vector>

// Define M_PI if it's not already defined by your GLM setup or other headers
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace GeometryHelpers {

    glm::vec3 getPerpendicularVector(const glm::vec3& p1, const glm::vec3& p2, float thickness);

    inline void pushVec3(std::vector<float>& target, const glm::vec3& v);

    void addQuad(std::vector<float>& target, const glm::vec3& p1, const glm::vec3& p2, float thickness);

    void addColoredQuad(std::vector<float>& target, const glm::vec3& p1, const glm::vec3& p2, float thickness, glm::vec3 clr);

    glm::vec3 get_log_jet_color(float value);

} // namespace GeometryHelpers

#endif
