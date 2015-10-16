#include "hvmc_collisions.h"

CollisionInfo::CollisionInfo()
{

}

CollisionInfo::~CollisionInfo()
{

}

bool CollisionInfo::Collide(RigidBody *a, RigidBody *b, const CollisionInfo &info)
{
    return false;
}
