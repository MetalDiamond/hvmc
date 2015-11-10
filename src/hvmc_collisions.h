#ifndef HVMC_COLLISIONS_H
#define HVMC_COLLISIONS_H

#include "hvmc_math.h"

struct RigidBody;

enum {SPHERE_TO_SPHERE, SPHERE_TO_BOX, BOX_TO_BOX};

struct CollisionInfo
{
    int type;
    vec2 intersection; // impact position
    // etc ...
};

class Collisions {

private:
    Collisions() {} // not instantiable (static class)

    static bool sphereToBox(RigidBody *sphere, RigidBody *box, CollisionInfo &info);
    static bool boxToBox(RigidBody *box1, RigidBody *box2, CollisionInfo &info);
    static bool sphereToSphere(RigidBody *sphere1, RigidBody *sphere2, CollisionInfo &info);

public:

    static bool Collide(RigidBody *a, RigidBody *b, CollisionInfo &info);
};

#endif

