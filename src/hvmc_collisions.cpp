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
    vec2 clamp_box = box->collider.dims/2;
    vec2 sphere_in_box_world = sphere->position-box->position;
    nearest.x = clamp(sphere_in_box_world.x,-clamp_box.x,clamp_box.x);
    nearest.y = clamp(sphere_in_box_world.y,-clamp_box.y,clamp_box.y);

    float dist = LengthSquared(sphere_in_box_world-nearest);
    float radius = sphere->collider.radius*sphere->collider.radius;

    if(dist < radius)
    {
        float realDist = sqrt(dist);
        info.intersection = (box->position + nearest) + (sphere_in_box_world*(realDist-sphere->collider.radius)/realDist)/2;
        info.normal = Normalize(nearest - sphere_in_box_world);
        info.type = SPHERE_TO_BOX;

        // Side of box collision
        if(abs(nearest.x) < abs(nearest.y))
            info.boxSideCol = SIDE_EDGE;
        else
            info.boxSideCol = UPPER_EGDE;

        return true;
    }
    else
        return false;
}

bool Collisions::boxToBox(RigidBody *box1, RigidBody *box2, CollisionInfo &info)
{
    vec2 diff = box1->position - box2->position;
    float diffx = fabs(diff.x) - (box1->collider.dims.x + box2->collider.dims.x)/2;
    float diffy = fabs(diff.y) - (box1->collider.dims.y + box2->collider.dims.y)/2;
    if(diffx < 0 && diffy < 0)
    {
        info.intersection = (box1->position + box2->position)/2;
        if(diffx > diffy)
            info.normal = {box1->position.x < box2->position.x ? -1 : 1, 0};
        else
            info.normal = {0, box1->position.y < box2->position.y ? -1 : 1};
        info.type = BOX_TO_BOX;
        return true;
    }
    return false;
}

bool Collisions::sphereToSphere(RigidBody *sphere1, RigidBody *sphere2, CollisionInfo &info)
{
    float dist = sphere1->collider.radius + sphere2->collider.radius;
    if(LengthSquared(sphere1->position - sphere2->position) < dist*dist)
    {
        vec2 pos1 = sphere1->position;
        vec2 pos2 = sphere2->position;

        info.intersection = vec2 {(pos1.x-pos2.x / 2), (pos1.y-pos2.y / 2)};
        info.normal = Normalize(pos1 - pos2);
        info.type = SPHERE_TO_SPHERE;

        return true;
    }
    else
        return false;
}
