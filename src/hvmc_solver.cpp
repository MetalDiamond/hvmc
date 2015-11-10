#include "hvmc_solver.h"

vec3 Constraint::getGradient(RigidBody* body)
{
    const float epsilon = 0.001f;
    vec3 gradient;
    float baseC = C();

    body->position.x += epsilon;
    gradient.x = C() - baseC;
    body->position.x -= epsilon;

    body->position.y += epsilon;
    gradient.y = C() - baseC;
    body->position.y -= epsilon;

    body->rotation += epsilon;
    gradient.z = C() - baseC;
    body->rotation -= epsilon;

    return gradient / epsilon;
}

SphereToSphereConstraint::SphereToSphereConstraint(RigidBody* sphere1, RigidBody* sphere2)
{
    // TODO
}

float SphereToSphereConstraint::C()
{
    return 0.5;
}


Solver::Solver()
{
}
