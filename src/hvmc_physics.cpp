#include "hvmc_physics.h"
#include "hvmc_world.h"
#include <iostream>

void RigidBody::Update( f32 dt )
{
}

void RigidBody::ApplyForce( vec2 const& f )
{
    forces += f;
}

void RigidBody::ApplyImpulse( vec2 const& impulse, vec2 const& contactVector )
{
    vec2 rayon = contactVector-position;
    //vec2 local = Normalize(rayon);
    //f32 intensity = Cross(local,impulse);

    velocity += impulse;//*intensity;
    angularVelocity += Cross(impulse, rayon);
}

void RigidBody::IntegrateVelocities(f32 dt) {
    position += velocity * dt;
    rotation += angularVelocity * dt;
}

void RigidBody::IntegrateForces(f32 dt) {
    velocity += forces * dt;
    forces = {0.0, 0.0};
    angularVelocity += torque * dt;
    torque = 0.0;
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
        body->ApplyForce(body->im * gravity);
    }

    //Gestion des bords
    for (unsigned int i=0; i<rigidBodies.size(); i++) {

        //DEBUG reset pos
        RigidBody *a = rigidBodies[i];

        vec2 pos = World::PhysicsToGraphicsPos(a->position);

        // loop

        while(pos.x > 800.f)
            pos.x -= 800.f;
        while(pos.y > 600.f)
            pos.y -= 600.f;
        while(pos.x < 0.f)
            pos.x += 800.f;
        while(pos.y < 0.f)
            pos.y += 600.f;

        a->position = World::GraphicsToPhysicsPos(pos);
    }

    for (unsigned int i=0; i<rigidBodies.size()-1; ++i)
    {
        for(unsigned int j=i+1; j<rigidBodies.size(); ++j)
        {
            RigidBody *a = rigidBodies[i];
            RigidBody *b = rigidBodies[j];
            if(a->im > 0 || b->im > 0) // if at least one of the two bodies is not kinematic
            {
                CollisionInfo colInfo;

                if (Collisions::Collide(a, b, colInfo)) {

                    //a->velocity = -1 * a->velocity;
                    //a->angularVelocity = - a->angularVelocity;
                    //b->velocity = -1 * b->velocity;
                    //b->angularVelocity = - b->angularVelocity;
                }
            }
        }
    }

    for(RigidBody* body : rigidBodies)
    {
        body->IntegrateForces(dt);
        body->IntegrateVelocities(dt);
    }
}

