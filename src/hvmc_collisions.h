#ifndef HVMC_COLLISIONS_H
#define HVMC_COLLISIONS_H

#include "hvmc_math.h"
//#include "hvmc_physics.h"

struct RigidBody;

class CollisionInfo {

private:

public:
    CollisionInfo();
    ~CollisionInfo();

    static bool Collide(RigidBody *a, RigidBody *b, const CollisionInfo &info);
};

//colision(circle, box);
//colision(circle, circle);
//colision(box, box);

#endif

