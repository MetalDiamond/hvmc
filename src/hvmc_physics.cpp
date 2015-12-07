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
    if(collider.type == RIGID_BODY_BOX)
        velocity += impulse;
    else
    {
        velocity += impulse;//*intensity;
        angularVelocity -= 0.3*Length(impulse)*Cross(contactVector-position, impulse);
    }
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

        pos.x -= 800 * floor(pos.x / 800);
        pos.y -= 600 * floor(pos.y / 600);

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

                if (Collisions::Collide(a, b, colInfo))
                {
                    // Génération de contraintes
                    if (colInfo.type == SPHERE_TO_SPHERE) {
                        system.pushConstraint(new SphereToSphereConstraint(i, j));

                    } else if (colInfo.type == SPHERE_TO_BOX) {
                        if(b->collider.type == RIGID_BODY_SPHERE)
                            system.pushConstraint(new BoxToSphereConstraint(i, j));
                        else
                            system.pushConstraint(new BoxToSphereConstraint(j, i));

                    } else if (colInfo.type == BOX_TO_BOX) {
                        system.pushConstraint(new BoxToBoxConstraint(i, j));
                    }

                    // Réaction générique quelle que soit le collider
                    const float damping = 0.8f;
                    float diff = Length(a->velocity - b->velocity)*damping;

                    if(a->im > 0)
                        a->ApplyImpulse(diff * colInfo.normal, colInfo.intersection);
                    if(b->im > 0)
                        b->ApplyImpulse(-diff * colInfo.normal, colInfo.intersection);
                }
            }
        }
    }

    // Résolution du système
    system.resolve(rigidBodies,5,dt);
    system.clear();

    for(RigidBody* body : rigidBodies)
    {
        body->IntegrateForces(dt);
        body->IntegrateVelocities(dt);
    }
}

