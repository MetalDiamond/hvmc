#include "hvmc_collisions.h"
#include "hvmc_physics.h"

bool Collisions::Collide(RigidBody *a, RigidBody *b, CollisionInfo &info)
{
    if(a->collider.type == RIGID_BODY_SPHERE)
    {
        if(b->collider.type == RIGID_BODY_SPHERE)
            return sphereToSphere(a, b, info);
        else
            return sphereToBox(a, b, info);
    }
    else
    {
        if(b->collider.type == RIGID_BODY_SPHERE)
            return sphereToBox(b, a, info);
        else
            return boxToBox(a, b, info);
    }
}

inline float clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

bool Collisions::sphereToBox(RigidBody *sphere, RigidBody *box, CollisionInfo &info)
{
    //translation
    vec2 nearest;
    vec2 for_clamp=box->collider.dims/2;
    vec2 sphere_in_box_world = sphere->position-box->position;
    nearest.x=clamp(sphere_in_box_world.x,-for_clamp.x,for_clamp.x);
    nearest.y=clamp(sphere_in_box_world.y,-for_clamp.y,for_clamp.y);

    float dist = LengthSquared(sphere_in_box_world-nearest);
    float radius = sphere->collider.radius*sphere->collider.radius;

    return dist < radius;
}

bool Collisions::boxToBox(RigidBody *box1, RigidBody *box2, CollisionInfo &info)
{
    vec2 diff = box1->position - box2->position;
    if(abs(diff.x) < (box1->collider.dims.x + box2->collider.dims.y)/2
            && abs(diff.y) < (box1->collider.dims.y + box2->collider.dims.y)/2)
    {
        // fill info
        return true;
    }
    return false;
}

bool Collisions::sphereToSphere(RigidBody *sphere1, RigidBody *sphere2, CollisionInfo &info)
{
    float dist = sphere1->collider.radius + sphere2->collider.radius;
    if(LengthSquared(sphere1->position - sphere2->position) < dist*dist)
    {
        // fill info
        return true;
    }
    else
        return false;
}
