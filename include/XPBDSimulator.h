#ifndef HAIR2D_XPBDSIMULATOR_H
#define HAIR2D_XPBDSIMULATOR_H

#include "Bundle.h"
#include "Collider.h"

class XPBDSimulator {
public:
    XPBDSimulator(Bundle* b_ptr, Collider* c_ptr);

    void addVel();

    void substep();

    inline void solveLengthConstraint(int i);

    inline void solveAngleConstraint(int i);

    inline void solveCollision(int i);

    inline void solveLineCollision(int i);


    int iterTimes{60};
    float dt{1.0f / (60.0f * 1.0f)};
    float damping{0.999f};
    float alpha{1e-12f};
    float lambda{0.0f};
    float alpha_angle{3.6e5f};
    float lambda0{.0f};
    float lambda1{.0f};

    // for collision
    float friction_coeff{0.3f};
    float restitution_coeff{0.2f};
    glm::vec3 m_gravity = glm::vec3(0.0f, -9.8f, 0.0f);
    Bundle* m_bundle{nullptr};
    Collider* m_collider{nullptr};
};




#endif
