#include "hvmc_physics.h"

void RigidBody::Update( f32 dt )
{
}

void RigidBody::ApplyForce( vec2 const& f )
{
    forces += f;
}

void RigidBody::ApplyImpulse( vec2 const& impulse, vec2 const& contactVector )
{

    velocity += Normalize(impulse)*Cross(impulse, Normalize(contactVector));
    vec2 rotated = {-contactVector.y, contactVector.x};
    angularVelocity += Cross(impulse, Normalize(rotated));
}

void RigidBody::SetKinematic()
{
    I = iI = m = im = 0.f;
}

bool PhysicsSystem::Init()
{
    gravity = vec2{ 0.f, -9.81f };

    return true;
}

void PhysicsSystem::Cleanup()
{
    rigidBodies.clear();
}

RigidBody* PhysicsSystem::AddSphere( vec2 const& pos, f32 radius )
{
    RigidBody* body = new RigidBody;
    
    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->iI = 1.f;
    body->position = pos;
    body->velocity = { 0.f, 0.f };

    body->collider.type = RIGID_BODY_SPHERE;
    body->collider.radius = radius;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddBox( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody; 
    
    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->position = pos;
    body->velocity = { 0.f, 0.f };
    
    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddWall( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody;

    body->im = 0.f;
    body->position = pos;

    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

void PhysicsSystem::Update( f32 dt )
{
    for(RigidBody* body : rigidBodies)
    {
        if(body->im != 0)
        {
            if(body->gravityMode)
                body->velocity += gravity*dt;
            body->velocity += body->forces*dt/body->im;
            body->angularVelocity += body->torque*dt/body->im;
        }
        body->position += body->velocity*dt;
        body->rotation += body->angularVelocity*dt;
    }
}

